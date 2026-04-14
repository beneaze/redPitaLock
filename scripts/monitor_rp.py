"""
monitor_rp.py -- Qt control GUI for the redPitaLock two-channel PID stabilizer.

Connects to the Red Pitaya daemon over TCP and provides:
  * Per-channel PID enable/disable (off by default)
  * Live adjustment of Kp, Ki, Kd, setpoint, loop rate, error sign, AOM LUT
  * Real-time pyqtgraph plots of analog input and output voltage

Usage:
    python monitor_rp.py                       # default host rp-XXXX.local
    python monitor_rp.py 192.168.1.100         # explicit IP
    python monitor_rp.py rp-XXXX.local 5000    # explicit host + port
"""

import sys
import socket
import numpy as np
from collections import deque

from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QDoubleSpinBox, QSpinBox, QComboBox,
    QLabel, QLineEdit, QStatusBar, QCheckBox,
)
from PySide6.QtGui import QFont
import pyqtgraph as pg

DEFAULT_HOST = "rp-XXXX.local"
DEFAULT_PORT = 5000
HISTORY = 400


# ------------------------------------------------------------------ TCP worker

class TcpWorker(QThread):
    """Background thread that maintains the TCP connection, parses telemetry,
    and forwards commands from the GUI."""

    data_received = Signal(int, float, float, float, float, int)  # ch, in_v, target_out, actual_out, sp, enabled
    autotune_done = Signal(int, float, float, float, float)       # ch, Ku, Tu, Kp, Ki
    autotune_failed = Signal(int)                                  # ch
    autotune_progress = Signal(int, int, int, float)               # ch, crossings, cycles, elapsed_s
    connected = Signal()
    disconnected = Signal(str)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self._running = True
        self._sock = None
        self._cmd_queue: list[str] = []

    def send_command(self, cmd: str):
        self._cmd_queue.append(cmd)

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def run(self):
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=5)
            self._sock.settimeout(0.05)
            self.connected.emit()
        except OSError as e:
            self.disconnected.emit(str(e))
            return

        buf = ""

        while self._running:
            # Send queued commands
            while self._cmd_queue:
                cmd = self._cmd_queue.pop(0)
                try:
                    self._sock.sendall((cmd + "\n").encode())
                except OSError:
                    self.disconnected.emit("send failed")
                    return

            # Read telemetry
            try:
                data = self._sock.recv(65536)
                if not data:
                    self.disconnected.emit("server closed connection")
                    return
                buf += data.decode("utf-8", errors="replace")
            except socket.timeout:
                continue
            except OSError:
                self.disconnected.emit("recv failed")
                return

            # Parse all complete lines; keep only the latest sample per channel
            # (server can send 100+ lines per recv — emitting each one floods the GUI).
            latest: dict[int, tuple] = {}
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                if line.startswith("D "):
                    parts = line.split()
                    if len(parts) >= 6:
                        try:
                            ch = int(parts[1])
                            iv = float(parts[2])
                            tgt = float(parts[3])
                            act = float(parts[4])
                            sp = float(parts[5])
                            en = int(parts[6]) if len(parts) > 6 else 0
                            latest[ch] = (iv, tgt, act, sp, en)
                        except (ValueError, IndexError):
                            pass
                elif line.startswith("AT "):
                    parts = line.split()
                    if len(parts) >= 5:
                        try:
                            self.autotune_progress.emit(
                                int(parts[1]), int(parts[2]),
                                int(parts[3]), float(parts[4]))
                        except (ValueError, IndexError):
                            pass
                elif line.startswith("AF "):
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            self.autotune_failed.emit(int(parts[1]))
                        except (ValueError, IndexError):
                            pass
                elif line.startswith("A "):
                    parts = line.split()
                    if len(parts) >= 6:
                        try:
                            ch = int(parts[1])
                            Ku = float(parts[2])
                            Tu = float(parts[3])
                            Kp = float(parts[4])
                            Ki = float(parts[5])
                            self.autotune_done.emit(ch, Ku, Tu, Kp, Ki)
                        except (ValueError, IndexError):
                            pass

            if latest:
                for ch in sorted(latest.keys()):
                    iv, tgt, act, sp, en = latest[ch]
                    self.data_received.emit(ch, iv, tgt, act, sp, en)

        if self._sock:
            self._sock.close()


# ----------------------------------------------------------- Channel widget

class ChannelPanel(QWidget):
    """Controls + plots for a single PID channel."""

    command = Signal(str)  # emitted when the user changes a parameter

    def __init__(self, ch_index: int, parent=None):
        super().__init__(parent)
        self.ch = ch_index
        self.input_data      = deque(maxlen=HISTORY)
        self.target_out_data = deque(maxlen=HISTORY)
        self.actual_out_data = deque(maxlen=HISTORY)
        self.sp_data         = deque(maxlen=HISTORY)
        self.x_data          = deque(maxlen=HISTORY)
        self.sample_idx      = 0
        self._autotuning     = False

        self._build_ui()

    # ......................................................... UI build

    def _build_ui(self):
        root = QHBoxLayout(self)

        # ---------- Left: controls ----------
        ctrl_box = QGroupBox(f"Channel {self.ch + 1} Controls")
        ctrl_layout = QGridLayout()
        ctrl_box.setLayout(ctrl_layout)
        row = 0

        # PID enable
        self.btn_enable = QPushButton("PID OFF")
        self.btn_enable.setCheckable(True)
        self.btn_enable.setChecked(False)
        self.btn_enable.setStyleSheet(
            "QPushButton { background-color: #cc3333; color: white; font-weight: bold; padding: 8px; }"
            "QPushButton:checked { background-color: #33aa33; }"
        )
        self.btn_enable.toggled.connect(self._on_enable_toggled)
        ctrl_layout.addWidget(self.btn_enable, row, 0, 1, 2)
        row += 1

        # Setpoint (fast ADC LV range is about +/-1 V)
        ctrl_layout.addWidget(QLabel("Setpoint (V)"), row, 0)
        self.sp_setpoint = QDoubleSpinBox()
        self.sp_setpoint.setRange(-1.0, 1.0)
        self.sp_setpoint.setDecimals(3)
        self.sp_setpoint.setSingleStep(0.01)
        self.sp_setpoint.setValue(0.5)
        self.sp_setpoint.setToolTip(
            "Target input voltage at the fast ADC (LV SMA ~ +/-1 V). "
            "Values outside that range cannot be reached."
        )
        self.sp_setpoint.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} setpoint {self.sp_setpoint.value():.4f}"))
        ctrl_layout.addWidget(self.sp_setpoint, row, 1)
        row += 1

        # PID output limits (controller command before DAC clamp +/-1 V)
        ctrl_layout.addWidget(QLabel("PID out min (V)"), row, 0)
        self.sp_out_min = QDoubleSpinBox()
        self.sp_out_min.setRange(-1.0, 1.0)
        self.sp_out_min.setDecimals(3)
        self.sp_out_min.setSingleStep(0.05)
        self.sp_out_min.setValue(0.0)
        self.sp_out_min.setToolTip("Minimum command the PID is allowed to request (volts to DAC path).")
        self.sp_out_min.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} out_min {self.sp_out_min.value():.4f}"))
        ctrl_layout.addWidget(self.sp_out_min, row, 1)
        row += 1

        ctrl_layout.addWidget(QLabel("PID out max (V)"), row, 0)
        self.sp_out_max = QDoubleSpinBox()
        self.sp_out_max.setRange(-1.0, 1.0)
        self.sp_out_max.setDecimals(3)
        self.sp_out_max.setSingleStep(0.05)
        self.sp_out_max.setValue(1.0)
        self.sp_out_max.setToolTip(
            "Maximum command the PID may request. Default 1.0 V matches STEMlab fast out. "
            "If the loop never reaches setpoint, raise this (and gains) or check error sign."
        )
        self.sp_out_max.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} out_max {self.sp_out_max.value():.4f}"))
        ctrl_layout.addWidget(self.sp_out_max, row, 1)
        row += 1

        # Kp
        ctrl_layout.addWidget(QLabel("Kp"), row, 0)
        self.sp_kp = QDoubleSpinBox()
        self.sp_kp.setRange(0.0, 1000.0)
        self.sp_kp.setDecimals(3)
        self.sp_kp.setSingleStep(0.1)
        self.sp_kp.setValue(4.0)
        self.sp_kp.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} kp {self.sp_kp.value():.4f}"))
        ctrl_layout.addWidget(self.sp_kp, row, 1)
        row += 1

        # Ki
        ctrl_layout.addWidget(QLabel("Ki"), row, 0)
        self.sp_ki = QDoubleSpinBox()
        self.sp_ki.setRange(0.0, 1000.0)
        self.sp_ki.setDecimals(3)
        self.sp_ki.setSingleStep(0.1)
        self.sp_ki.setValue(2.0)
        self.sp_ki.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} ki {self.sp_ki.value():.4f}"))
        ctrl_layout.addWidget(self.sp_ki, row, 1)
        row += 1

        # Kd
        ctrl_layout.addWidget(QLabel("Kd"), row, 0)
        self.sp_kd = QDoubleSpinBox()
        self.sp_kd.setRange(0.0, 100.0)
        self.sp_kd.setDecimals(4)
        self.sp_kd.setSingleStep(0.001)
        self.sp_kd.setValue(0.0)
        self.sp_kd.setToolTip(
            "Derivative gain. Uses rate of change of the *photodiode voltage*, not raw error, "
            "so ADC noise is not amplified. Start at 0; add a little only if you need damping."
        )
        self.sp_kd.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} kd {self.sp_kd.value():.5f}"))
        ctrl_layout.addWidget(self.sp_kd, row, 1)
        row += 1

        # Loop rate
        ctrl_layout.addWidget(QLabel("Loop period (us)"), row, 0)
        self.sp_loop = QSpinBox()
        self.sp_loop.setRange(10, 1000000)
        self.sp_loop.setSingleStep(100)
        self.sp_loop.setValue(1000)
        self.sp_loop.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} loop_rate {self.sp_loop.value()}"))
        ctrl_layout.addWidget(self.sp_loop, row, 1)
        row += 1

        # Error sign
        ctrl_layout.addWidget(QLabel("Error sign"), row, 0)
        self.cb_sign = QComboBox()
        self.cb_sign.addItems(["+1 (normal)", "-1 (inverted)"])
        self.cb_sign.currentIndexChanged.connect(self._on_sign_changed)
        ctrl_layout.addWidget(self.cb_sign, row, 1)
        row += 1

        # AOM LUT
        self.chk_lut = QCheckBox("AOM linearization (LUT)")
        self.chk_lut.setChecked(False)
        self.chk_lut.toggled.connect(
            lambda v: self.command.emit(f"SET {self.ch} use_lut {int(v)}"))
        ctrl_layout.addWidget(self.chk_lut, row, 0, 1, 2)
        row += 1

        # Reset PID
        btn_reset = QPushButton("Reset PID integrator")
        btn_reset.clicked.connect(
            lambda: self.command.emit(f"SET {self.ch} reset 0"))
        ctrl_layout.addWidget(btn_reset, row, 0, 1, 2)
        row += 1

        # Autotune relay amplitude
        ctrl_layout.addWidget(QLabel("Autotune amp (V)"), row, 0)
        self.sp_at_amp = QDoubleSpinBox()
        self.sp_at_amp.setRange(0.01, 1.0)
        self.sp_at_amp.setDecimals(3)
        self.sp_at_amp.setSingleStep(0.05)
        self.sp_at_amp.setValue(0.50)
        self.sp_at_amp.setToolTip(
            "Half-amplitude of the relay output during autotune. "
            "With out_min=0 and out_max=1, amp=0.5 uses the full range [0,1]."
        )
        self.sp_at_amp.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} autotune_amp {self.sp_at_amp.value():.4f}"))
        ctrl_layout.addWidget(self.sp_at_amp, row, 1)
        row += 1

        # Autotune button
        self.btn_autotune = QPushButton("Autotune")
        self.btn_autotune.setStyleSheet(
            "QPushButton { background-color: #1565C0; color: white; font-weight: bold; padding: 8px; }"
        )
        self.btn_autotune.clicked.connect(self._on_autotune_clicked)
        ctrl_layout.addWidget(self.btn_autotune, row, 0, 1, 2)
        row += 1

        ctrl_layout.setRowStretch(row, 1)
        ctrl_box.setFixedWidth(280)

        # ---------- Right: plots ----------
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)
        plot_layout.setContentsMargins(0, 0, 0, 0)

        pg.setConfigOptions(antialias=True)

        self.plot_input = pg.PlotWidget(title=f"Ch{self.ch+1} Analog Input")
        self.plot_input.setLabel("left", "Voltage", units="V")
        self.plot_input.setLabel("bottom", "Sample")
        self.plot_input.addLegend(offset=(10, 10))
        self.curve_input = self.plot_input.plot(pen=pg.mkPen("#2196F3", width=2), name="Input")
        self.curve_sp    = self.plot_input.plot(pen=pg.mkPen("#4CAF50", width=2, style=Qt.DashLine), name="Setpoint")
        for c in (self.curve_input, self.curve_sp):
            c.setDownsampling(auto=True, method="peak")
            c.setClipToView(True)

        self.plot_output = pg.PlotWidget(title=f"Ch{self.ch+1} Output Voltage")
        self.plot_output.setLabel("left", "Voltage", units="V")
        self.plot_output.setLabel("bottom", "Sample")
        self.plot_output.addLegend(offset=(10, 10))
        self.curve_target_out = self.plot_output.plot(pen=pg.mkPen("#FF5722", width=2, style=Qt.DashLine), name="Target")
        self.curve_actual_out = self.plot_output.plot(pen=pg.mkPen("#E91E63", width=2), name="Actual")
        for c in (self.curve_target_out, self.curve_actual_out):
            c.setDownsampling(auto=True, method="peak")
            c.setClipToView(True)

        plot_layout.addWidget(self.plot_input)
        plot_layout.addWidget(self.plot_output)

        root.addWidget(ctrl_box)
        root.addWidget(plot_widget, stretch=1)

    # ...................................................... callbacks

    def _on_enable_toggled(self, checked):
        self.btn_enable.setText("PID ON" if checked else "PID OFF")
        self.command.emit(f"SET {self.ch} enabled {int(checked)}")

    def _on_sign_changed(self, idx):
        sign = 1.0 if idx == 0 else -1.0
        self.command.emit(f"SET {self.ch} error_sign {sign:.1f}")

    def _on_autotune_clicked(self):
        if self._autotuning:
            self._autotuning = False
            self.btn_autotune.setText("Autotune")
            self.command.emit(f"SET {self.ch} autotune 0")
            return
        if not self.btn_enable.isChecked():
            return
        self._autotuning = True
        self.btn_autotune.setText("Cancel Autotune")
        self.command.emit(f"SET {self.ch} autotune 1")

    @Slot(float, float, float, float)
    def on_autotune_done(self, Ku, Tu, Kp, Ki):
        self._autotuning = False
        self.sp_kp.setValue(Kp)
        self.sp_ki.setValue(Ki)
        self.sp_kd.setValue(0.0)
        self.btn_autotune.setText("Autotune")
        return Ku, Tu

    @Slot()
    def on_autotune_failed(self):
        self._autotuning = False
        self.btn_autotune.setText("Autotune")

    @Slot(int, int, float)
    def on_autotune_progress(self, crossings, cycles, elapsed):
        if self._autotuning:
            self.btn_autotune.setText(
                f"Cancel ({crossings} cross, {cycles} cyc, {elapsed:.0f}s)")

    # ...................................................... data feed

    @Slot(float, float, float, float, int)
    def add_sample(self, input_v, target_out_v, actual_out_v, setpoint_v, enabled):
        self.input_data.append(input_v)
        self.target_out_data.append(target_out_v)
        self.actual_out_data.append(actual_out_v)
        self.sp_data.append(setpoint_v)
        self.x_data.append(self.sample_idx)
        self.sample_idx += 1

    def refresh_plots(self):
        if not self.x_data:
            return
        x = np.array(self.x_data)
        self.curve_input.setData(x, np.array(self.input_data))
        self.curve_sp.setData(x, np.array(self.sp_data))
        self.curve_target_out.setData(x, np.array(self.target_out_data))
        self.curve_actual_out.setData(x, np.array(self.actual_out_data))


# ----------------------------------------------------------- Main window

class MainWindow(QMainWindow):
    def __init__(self, host: str, port: int):
        super().__init__()
        self.setWindowTitle("redPitaLock -- PID Stabilizer Control")
        self.resize(1100, 700)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Connection bar
        conn_bar = QHBoxLayout()
        conn_bar.addWidget(QLabel("Host:"))
        self.txt_host = QLineEdit(host)
        self.txt_host.setFixedWidth(200)
        conn_bar.addWidget(self.txt_host)
        conn_bar.addWidget(QLabel("Port:"))
        self.txt_port = QLineEdit(str(port))
        self.txt_port.setFixedWidth(60)
        conn_bar.addWidget(self.txt_port)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._on_connect)
        conn_bar.addWidget(self.btn_connect)
        conn_bar.addStretch()
        layout.addLayout(conn_bar)

        # Tabs for channels
        self.tabs = QTabWidget()
        self.channel_panels: list[ChannelPanel] = []
        for i in range(2):
            panel = ChannelPanel(i)
            self.channel_panels.append(panel)
            self.tabs.addTab(panel, f"Channel {i + 1}")
        layout.addWidget(self.tabs)

        # Status bar
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Disconnected")

        # TCP worker
        self.worker: TcpWorker | None = None

        # Plot refresh timer (30 Hz)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(33)

    # ......................................................... connect

    def _on_connect(self):
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(2000)
            self.worker = None
            self.btn_connect.setText("Connect")
            self.status.showMessage("Disconnected")
            return

        host = self.txt_host.text().strip()
        port = int(self.txt_port.text().strip())

        self.worker = TcpWorker(host, port)
        self.worker.data_received.connect(self._on_data)
        self.worker.autotune_done.connect(self._on_autotune_done)
        self.worker.autotune_failed.connect(self._on_autotune_failed)
        self.worker.autotune_progress.connect(self._on_autotune_progress)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)

        for panel in self.channel_panels:
            panel.command.connect(self._send_command)

        self.status.showMessage(f"Connecting to {host}:{port}...")
        self.worker.start()

    @Slot()
    def _on_connected(self):
        self.status.showMessage(
            f"Connected to {self.txt_host.text()}:{self.txt_port.text()}")
        self.btn_connect.setText("Disconnect")

    @Slot(str)
    def _on_disconnected(self, reason):
        self.status.showMessage(f"Disconnected: {reason}")
        self.btn_connect.setText("Connect")

    @Slot(str)
    def _send_command(self, cmd):
        if self.worker:
            self.worker.send_command(cmd)

    @Slot(int, float, float, float, float, int)
    def _on_data(self, ch, input_v, target_out_v, actual_out_v, setpoint_v, enabled):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].add_sample(input_v, target_out_v, actual_out_v, setpoint_v, enabled)

    @Slot(int, float, float, float, float)
    def _on_autotune_done(self, ch, Ku, Tu, Kp, Ki):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].on_autotune_done(Ku, Tu, Kp, Ki)
            self.status.showMessage(
                f"Ch{ch+1} autotune complete: Ku={Ku:.4f} Tu={Tu:.4f}s  →  Kp={Kp:.4f} Ki={Ki:.4f}")

    @Slot(int)
    def _on_autotune_failed(self, ch):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].on_autotune_failed()
            self.status.showMessage(
                f"Ch{ch+1} autotune FAILED (timeout) — try increasing relay amplitude or check setpoint")

    @Slot(int, int, int, float)
    def _on_autotune_progress(self, ch, crossings, cycles, elapsed):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].on_autotune_progress(crossings, cycles, elapsed)

    def _refresh_plots(self):
        for panel in self.channel_panels:
            panel.refresh_plots()

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
            self.worker.wait(2000)
        event.accept()


# ------------------------------------------------------------------ main

def main():
    host = DEFAULT_HOST
    port = DEFAULT_PORT

    if len(sys.argv) >= 2:
        host = sys.argv[1]
    if len(sys.argv) >= 3:
        port = int(sys.argv[2])

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    font = QFont()
    font.setPointSize(10)
    app.setFont(font)

    window = MainWindow(host, port)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

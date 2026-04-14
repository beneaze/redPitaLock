"""
monitor_rp.py -- Qt control GUI for the redPitaLock two-channel PID stabilizer.

Connects to the Red Pitaya daemon over TCP and provides:
  * Per-channel PID enable/disable (off by default)
  * Live adjustment of Kp, Ki, Kd, setpoint, loop rate, error sign, AOM LUT
  * Real-time pyqtgraph plots of analog input and output voltage vs time
  * Optional CSV recording of input (photodiode) voltage vs time for stability checks
  * Optional histogram of input with Gaussian overlay and mean / std (View menu)

Usage:
    python monitor_rp.py                       # default host rp-XXXX.local
    python monitor_rp.py 192.168.1.100         # explicit IP
    python monitor_rp.py rp-XXXX.local 5000    # explicit host + port
"""

import csv
import socket
import sys
import time
from collections import deque
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtGui import QAction, QFont
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QWidgetAction,
)
import pyqtgraph as pg

DEFAULT_HOST = "rp-XXXX.local"
DEFAULT_PORT = 5000
HISTORY = 400
DEFAULT_HISTORY_SECONDS = 10.0


# ------------------------------------------------------------------ TCP worker

class TcpWorker(QThread):
    """Background thread that maintains the TCP connection, parses telemetry,
    and forwards commands from the GUI."""

    data_received = Signal(int, float, float, float, float, float, int)  # ch, time_s, in_v, target_out, actual_out, sp, enabled
    params_received = Signal(int, dict)                                  # ch, {key: value_str, ...}
    psd_received = Signal(int, int, float, object)                       # ch, n_bins, fs, np.array of PSD bins
    autotune_done = Signal(int, float, float, float, float)              # ch, Ku, Tu, Kp, Ki
    autotune_failed = Signal(int)                                        # ch
    autotune_progress = Signal(int, int, int, float)                     # ch, crossings, cycles, elapsed_s
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
            pending_psd_hdr = None  # (ch, n_bins, fs) waiting for data line
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                # PSD data line (follows a PSD header)
                if pending_psd_hdr is not None:
                    p_ch, p_nbins, p_fs = pending_psd_hdr
                    pending_psd_hdr = None
                    try:
                        bins = np.fromstring(line, dtype=float, sep=" ")
                        if bins.size == p_nbins:
                            self.psd_received.emit(p_ch, p_nbins, p_fs, bins)
                    except Exception:
                        pass
                    continue

                if line.startswith("D "):
                    parts = line.split()
                    if len(parts) >= 7:
                        try:
                            ch  = int(parts[1])
                            ts  = float(parts[2])
                            iv  = float(parts[3])
                            tgt = float(parts[4])
                            act = float(parts[5])
                            sp  = float(parts[6])
                            en  = int(parts[7]) if len(parts) > 7 else 0
                            latest[ch] = (ts, iv, tgt, act, sp, en)
                        except (ValueError, IndexError):
                            pass
                elif line.startswith("PSD "):
                    parts = line.split()
                    if len(parts) >= 4:
                        try:
                            pending_psd_hdr = (
                                int(parts[1]),
                                int(parts[2]),
                                float(parts[3]),
                            )
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
                elif line.startswith("P "):
                    parts = line.split(None, 2)
                    if len(parts) >= 3:
                        try:
                            p_ch = int(parts[1])
                            kv = {}
                            for token in parts[2].split():
                                if "=" in token:
                                    k, v = token.split("=", 1)
                                    kv[k] = v
                            self.params_received.emit(p_ch, kv)
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
                    ts, iv, tgt, act, sp, en = latest[ch]
                    self.data_received.emit(ch, ts, iv, tgt, act, sp, en)

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
        self.t_data          = deque(maxlen=HISTORY)  # server-side timestamp (s)
        self._autotuning     = False
        self._stats_visible  = False
        self._psd_visible    = False

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

        # --- Manual output / waveform (active when PID is off) ---
        self.manual_group = QGroupBox("Manual Output (PID off)")
        manual_layout = QGridLayout()
        self.manual_group.setLayout(manual_layout)
        mrow = 0

        manual_layout.addWidget(QLabel("Mode"), mrow, 0)
        self.cb_out_mode = QComboBox()
        self.cb_out_mode.addItems(["DC", "Triangle", "Sine"])
        self.cb_out_mode.currentIndexChanged.connect(self._on_out_mode_changed)
        manual_layout.addWidget(self.cb_out_mode, mrow, 1)
        mrow += 1

        manual_layout.addWidget(QLabel("DC voltage (V)"), mrow, 0)
        self.sp_manual_v = QDoubleSpinBox()
        self.sp_manual_v.setRange(-1.0, 1.0)
        self.sp_manual_v.setDecimals(4)
        self.sp_manual_v.setSingleStep(0.01)
        self.sp_manual_v.setValue(0.0)
        self.sp_manual_v.setToolTip("Output voltage when mode is DC and PID is off.")
        self.sp_manual_v.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} manual_v {self.sp_manual_v.value():.4f}"))
        manual_layout.addWidget(self.sp_manual_v, mrow, 1)
        mrow += 1

        manual_layout.addWidget(QLabel("Frequency (Hz)"), mrow, 0)
        self.sp_wave_freq = QDoubleSpinBox()
        self.sp_wave_freq.setRange(0.001, 10000.0)
        self.sp_wave_freq.setDecimals(3)
        self.sp_wave_freq.setSingleStep(0.1)
        self.sp_wave_freq.setValue(1.0)
        self.sp_wave_freq.setToolTip("Waveform frequency for triangle / sine mode.")
        self.sp_wave_freq.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} wave_freq {self.sp_wave_freq.value():.4f}"))
        manual_layout.addWidget(self.sp_wave_freq, mrow, 1)
        mrow += 1

        manual_layout.addWidget(QLabel("Amplitude (V)"), mrow, 0)
        self.sp_wave_amp = QDoubleSpinBox()
        self.sp_wave_amp.setRange(0.0, 1.0)
        self.sp_wave_amp.setDecimals(4)
        self.sp_wave_amp.setSingleStep(0.01)
        self.sp_wave_amp.setValue(0.5)
        self.sp_wave_amp.setToolTip("Peak amplitude of the waveform (V).")
        self.sp_wave_amp.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} wave_amp {self.sp_wave_amp.value():.4f}"))
        manual_layout.addWidget(self.sp_wave_amp, mrow, 1)
        mrow += 1

        manual_layout.addWidget(QLabel("Offset (V)"), mrow, 0)
        self.sp_wave_offset = QDoubleSpinBox()
        self.sp_wave_offset.setRange(-1.0, 1.0)
        self.sp_wave_offset.setDecimals(4)
        self.sp_wave_offset.setSingleStep(0.01)
        self.sp_wave_offset.setValue(0.0)
        self.sp_wave_offset.setToolTip("DC offset added to the waveform (V).")
        self.sp_wave_offset.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} wave_offset {self.sp_wave_offset.value():.4f}"))
        manual_layout.addWidget(self.sp_wave_offset, mrow, 1)
        mrow += 1

        ctrl_layout.addWidget(self.manual_group, row, 0, 1, 2)
        row += 1

        self._update_manual_fields_visibility()

        ctrl_layout.setRowStretch(row, 1)
        ctrl_box.setFixedWidth(280)

        # ---------- Right: plots ----------
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)
        plot_layout.setContentsMargins(0, 0, 0, 0)

        pg.setConfigOptions(antialias=True)

        self.plot_input = pg.PlotWidget(title=f"Ch{self.ch+1} Analog Input")
        self.plot_input.setLabel("left", "Voltage", units="V")
        self.plot_input.setLabel("bottom", "Time", units="s")
        self.plot_input.addLegend(offset=(10, 10))
        self.curve_input = self.plot_input.plot(pen=pg.mkPen("#2196F3", width=2), name="Input")
        self.curve_sp    = self.plot_input.plot(pen=pg.mkPen("#4CAF50", width=2, style=Qt.DashLine), name="Setpoint")
        for c in (self.curve_input, self.curve_sp):
            c.setDownsampling(auto=True, method="peak")
            c.setClipToView(True)

        self.plot_output = pg.PlotWidget(title=f"Ch{self.ch+1} Output Voltage")
        self.plot_output.setLabel("left", "Voltage", units="V")
        self.plot_output.setLabel("bottom", "Time", units="s")
        self.plot_output.addLegend(offset=(10, 10))
        self.curve_target_out = self.plot_output.plot(pen=pg.mkPen("#FF5722", width=2, style=Qt.DashLine), name="Target")
        self.curve_actual_out = self.plot_output.plot(pen=pg.mkPen("#E91E63", width=2), name="Actual")
        for c in (self.curve_target_out, self.curve_actual_out):
            c.setDownsampling(auto=True, method="peak")
            c.setClipToView(True)

        self.plot_stats = pg.PlotWidget(title=f"Ch{self.ch+1} Input distribution (stability)")
        self.plot_stats.setLabel("left", "Count")
        self.plot_stats.setLabel("bottom", "Input voltage", units="V")
        self._bar_item = pg.BarGraphItem(x=[], height=[], width=0, brush="#42A5F5")
        self.plot_stats.addItem(self._bar_item)
        self.curve_gauss = self.plot_stats.plot(
            pen=pg.mkPen("#E65100", width=2), name="Gaussian fit")
        self.plot_stats.addLegend(offset=(10, 10))
        self.plot_stats.hide()

        self.plot_psd = pg.PlotWidget(title=f"Ch{self.ch+1} Power Spectral Density")
        self.plot_psd.setLabel("left", "PSD", units="V^2/Hz")
        self.plot_psd.setLabel("bottom", "Frequency", units="Hz")
        self.plot_psd.setLogMode(x=True, y=True)
        self.curve_psd = self.plot_psd.plot(pen=pg.mkPen("#7B1FA2", width=2))
        self.plot_psd.hide()

        plot_layout.addWidget(self.plot_input)
        plot_layout.addWidget(self.plot_output)
        plot_layout.addWidget(self.plot_stats)
        plot_layout.addWidget(self.plot_psd)

        root.addWidget(ctrl_box)
        root.addWidget(plot_widget, stretch=1)

    # ...................................................... param sync

    def sync_from_params(self, kv: dict):
        """Update all widgets from a key=value dict received from the server.
        Blocks signals so we don't re-emit SET commands back to the device."""
        def _f(v): return float(v)
        def _i(v): return int(float(v))

        for widget in (
            self.btn_enable, self.sp_setpoint, self.sp_out_min, self.sp_out_max,
            self.sp_kp, self.sp_ki, self.sp_kd, self.sp_loop, self.cb_sign,
            self.chk_lut, self.cb_out_mode, self.sp_manual_v, self.sp_wave_freq,
            self.sp_wave_amp, self.sp_wave_offset, self.sp_at_amp,
        ):
            widget.blockSignals(True)

        if "enabled" in kv:
            en = _i(kv["enabled"])
            self.btn_enable.setChecked(bool(en))
            self.btn_enable.setText("PID ON" if en else "PID OFF")
        if "setpoint" in kv:
            self.sp_setpoint.setValue(_f(kv["setpoint"]))
        if "out_min" in kv:
            self.sp_out_min.setValue(_f(kv["out_min"]))
        if "out_max" in kv:
            self.sp_out_max.setValue(_f(kv["out_max"]))
        if "kp" in kv:
            self.sp_kp.setValue(_f(kv["kp"]))
        if "ki" in kv:
            self.sp_ki.setValue(_f(kv["ki"]))
        if "kd" in kv:
            self.sp_kd.setValue(_f(kv["kd"]))
        if "loop_rate" in kv:
            self.sp_loop.setValue(_i(kv["loop_rate"]))
        if "error_sign" in kv:
            self.cb_sign.setCurrentIndex(0 if _f(kv["error_sign"]) >= 0 else 1)
        if "use_lut" in kv:
            self.chk_lut.setChecked(bool(_i(kv["use_lut"])))
        if "out_mode" in kv:
            self.cb_out_mode.setCurrentIndex(_i(kv["out_mode"]))
        if "manual_v" in kv:
            self.sp_manual_v.setValue(_f(kv["manual_v"]))
        if "wave_freq" in kv:
            self.sp_wave_freq.setValue(_f(kv["wave_freq"]))
        if "wave_amp" in kv:
            self.sp_wave_amp.setValue(_f(kv["wave_amp"]))
        if "wave_offset" in kv:
            self.sp_wave_offset.setValue(_f(kv["wave_offset"]))
        if "integral_max" in kv:
            pass

        for widget in (
            self.btn_enable, self.sp_setpoint, self.sp_out_min, self.sp_out_max,
            self.sp_kp, self.sp_ki, self.sp_kd, self.sp_loop, self.cb_sign,
            self.chk_lut, self.cb_out_mode, self.sp_manual_v, self.sp_wave_freq,
            self.sp_wave_amp, self.sp_wave_offset, self.sp_at_amp,
        ):
            widget.blockSignals(False)

        self._update_manual_fields_visibility()

    # ...................................................... callbacks

    def _on_enable_toggled(self, checked):
        self.btn_enable.setText("PID ON" if checked else "PID OFF")
        self.command.emit(f"SET {self.ch} enabled {int(checked)}")
        self._update_manual_fields_visibility()

    def _on_out_mode_changed(self, idx):
        self.command.emit(f"SET {self.ch} out_mode {idx}")
        self._update_manual_fields_visibility()

    def _update_manual_fields_visibility(self):
        pid_on = self.btn_enable.isChecked()
        self.manual_group.setEnabled(not pid_on)
        mode = self.cb_out_mode.currentIndex()
        is_wave = mode in (1, 2)
        self.sp_manual_v.setEnabled(not pid_on and not is_wave)
        self.sp_wave_freq.setEnabled(not pid_on and is_wave)
        self.sp_wave_amp.setEnabled(not pid_on and is_wave)
        self.sp_wave_offset.setEnabled(not pid_on and is_wave)

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

    def set_history_size(self, n: int):
        """Resize all rolling buffers to hold *n* samples."""
        self.input_data      = deque(self.input_data, maxlen=n)
        self.target_out_data = deque(self.target_out_data, maxlen=n)
        self.actual_out_data = deque(self.actual_out_data, maxlen=n)
        self.sp_data         = deque(self.sp_data, maxlen=n)
        self.t_data          = deque(self.t_data, maxlen=n)

    def reset_timebase(self):
        """Clear history and restart the time axis (e.g. on new TCP connection)."""
        self.input_data.clear()
        self.target_out_data.clear()
        self.actual_out_data.clear()
        self.sp_data.clear()
        self.t_data.clear()

    def set_statistics_visible(self, visible: bool):
        self._stats_visible = visible
        self.plot_stats.setVisible(visible)
        if visible:
            self._refresh_stats_plot()

    def set_psd_visible(self, visible: bool):
        self._psd_visible = visible
        self.plot_psd.setVisible(visible)

    @Slot(float, float, float, float, float, int)
    def add_sample(self, time_s, input_v, target_out_v, actual_out_v, setpoint_v, enabled):
        self.t_data.append(time_s)
        self.input_data.append(input_v)
        self.target_out_data.append(target_out_v)
        self.actual_out_data.append(actual_out_v)
        self.sp_data.append(setpoint_v)

    def set_psd_data(self, n_bins: int, fs: float, bins):
        if not self._psd_visible:
            return
        freqs = np.arange(n_bins) * (fs / ((n_bins - 1) * 2))
        mask = freqs > 0
        self.curve_psd.setData(freqs[mask], bins[mask])
        self.plot_psd.setTitle(
            f"Ch{self.ch+1} PSD  (fs={fs:.0f} Hz, {n_bins} bins)"
        )

    def _refresh_stats_plot(self):
        data = np.asarray(self.input_data, dtype=float)
        if data.size < 8:
            self.plot_stats.setTitle(
                f"Ch{self.ch+1} Input distribution — collect more samples (need ≥8)"
            )
            self._bar_item.setOpts(x=[], height=[], width=0.01)
            self.curve_gauss.setData([], [])
            return
        mu = float(np.mean(data))
        sigma = float(np.std(data, ddof=1)) if data.size > 1 else 0.0
        if sigma <= 1e-12:
            sigma = 1e-12
        nbins = int(np.clip(int(np.sqrt(len(data))), 10, 40))
        hist, edges = np.histogram(data, bins=nbins)
        centers = (edges[:-1] + edges[1:]) / 2.0
        width = float(edges[1] - edges[0])
        self._bar_item.setOpts(
            x=centers, height=hist, width=width * 0.9, brush="#42A5F5"
        )
        xg = np.linspace(mu - 4.0 * sigma, mu + 4.0 * sigma, 200)
        pdf = (1.0 / (sigma * np.sqrt(2.0 * np.pi))) * np.exp(
            -0.5 * ((xg - mu) / sigma) ** 2
        )
        yg = data.size * width * pdf
        self.curve_gauss.setData(xg, yg)
        self.plot_stats.setTitle(
            f"Ch{self.ch+1} Input: μ={mu:.5f} V  σ={sigma:.5f} V  (N={data.size})"
        )

    def refresh_plots(self):
        if not self.t_data:
            return
        x = np.array(self.t_data, dtype=float)
        self.curve_input.setData(x, np.array(self.input_data))
        self.curve_sp.setData(x, np.array(self.sp_data))
        self.curve_target_out.setData(x, np.array(self.target_out_data))
        self.curve_actual_out.setData(x, np.array(self.actual_out_data))
        if self._stats_visible:
            self._refresh_stats_plot()


# ----------------------------------------------------------- Main window

class MainWindow(QMainWindow):
    def __init__(self, host: str, port: int):
        super().__init__()
        self.setWindowTitle("redPitaLock -- PID Stabilizer Control")
        self.resize(1200, 880)

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

        # Menu bar
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("&File")
        self.act_record_start = QAction("Start &recording trace…", self)
        self.act_record_start.setToolTip(
            "Append input (photodiode) voltages vs time to a CSV file for stability analysis."
        )
        self.act_record_start.triggered.connect(self._start_recording)
        file_menu.addAction(self.act_record_start)
        self.act_record_stop = QAction("S&top recording", self)
        self.act_record_stop.setEnabled(False)
        self.act_record_stop.triggered.connect(self._stop_recording)
        file_menu.addAction(self.act_record_stop)

        view_menu = menu_bar.addMenu("&View")
        self.act_stats = QAction("Input &statistics (histogram)", self)
        self.act_stats.setCheckable(True)
        self.act_stats.setToolTip(
            "Histogram of fast ADC input with Gaussian overlay, mean μ and std σ."
        )
        self.act_stats.toggled.connect(self._on_stats_toggled)
        view_menu.addAction(self.act_stats)

        self.act_psd = QAction("Power spectral &density", self)
        self.act_psd.setCheckable(True)
        self.act_psd.setToolTip(
            "On-chip FFT power spectral density (V²/Hz) computed on the Red Pitaya."
        )
        self.act_psd.toggled.connect(self._on_psd_toggled)
        view_menu.addAction(self.act_psd)
        view_menu.addSeparator()

        history_widget = QWidget()
        history_layout = QHBoxLayout(history_widget)
        history_layout.setContentsMargins(8, 2, 8, 2)
        history_layout.addWidget(QLabel("History window (s):"))
        self.sp_history_sec = QDoubleSpinBox()
        self.sp_history_sec.setRange(1.0, 600.0)
        self.sp_history_sec.setDecimals(1)
        self.sp_history_sec.setSingleStep(1.0)
        self.sp_history_sec.setValue(DEFAULT_HISTORY_SECONDS)
        self.sp_history_sec.setToolTip(
            "Length of the rolling plot window in seconds. "
            "Internally converted to a sample count based on the measured telemetry rate."
        )
        self.sp_history_sec.editingFinished.connect(self._on_history_changed)
        history_layout.addWidget(self.sp_history_sec)
        history_action = QWidgetAction(self)
        history_action.setDefaultWidget(history_widget)
        view_menu.addAction(history_action)
        self._history_seconds = DEFAULT_HISTORY_SECONDS

        self._record_file = None
        self._record_writer = None
        self._record_t0: Optional[float] = None
        self._last_input: list[Optional[float]] = [None, None]

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
        self.worker.params_received.connect(self._on_params)
        self.worker.psd_received.connect(self._on_psd)
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
        self._last_input = [None, None]
        for panel in self.channel_panels:
            panel.reset_timebase()
        for ch in range(len(self.channel_panels)):
            self.worker.send_command(f"GET {ch} params")
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

    @Slot(int, dict)
    def _on_params(self, ch, kv):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].sync_from_params(kv)

    @Slot(int, float, float, float, float, float, int)
    def _on_data(self, ch, time_s, input_v, target_out_v, actual_out_v, setpoint_v, enabled):
        self._last_input[ch] = input_v
        if self._record_writer is not None and self._record_t0 is not None:
            t = time.perf_counter() - self._record_t0
            self._record_writer.writerow(
                [
                    f"{t:.6f}",
                    ""
                    if self._last_input[0] is None
                    else f"{self._last_input[0]:.6f}",
                    ""
                    if self._last_input[1] is None
                    else f"{self._last_input[1]:.6f}",
                ]
            )
            self._record_file.flush()
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].add_sample(
                time_s, input_v, target_out_v, actual_out_v, setpoint_v, enabled
            )

    @Slot(int, int, float, object)
    def _on_psd(self, ch, n_bins, fs, bins):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].set_psd_data(n_bins, fs, bins)

    @Slot(bool)
    def _on_stats_toggled(self, checked: bool):
        for panel in self.channel_panels:
            panel.set_statistics_visible(checked)

    @Slot(bool)
    def _on_psd_toggled(self, checked: bool):
        for panel in self.channel_panels:
            panel.set_psd_visible(checked)

    def _on_history_changed(self):
        secs = self.sp_history_sec.value()
        self._history_seconds = secs
        for panel in self.channel_panels:
            if len(panel.t_data) >= 2:
                dt = panel.t_data[-1] - panel.t_data[0]
                rate = (len(panel.t_data) - 1) / dt if dt > 0 else 100.0
            else:
                rate = 100.0
            n = max(16, int(secs * rate))
            panel.set_history_size(n)
        self.status.showMessage(f"History window set to {secs:.1f} s")

    def _start_recording(self):
        if self._record_writer is not None:
            self.status.showMessage("Already recording — stop first.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save input trace (CSV)",
            "",
            "CSV (*.csv);;All files (*)",
        )
        if not path:
            return
        try:
            f = open(path, "w", newline="", encoding="utf-8")
        except OSError as e:
            self.status.showMessage(f"Could not open file: {e}")
            return
        self._record_file = f
        self._record_writer = csv.writer(f)
        self._record_writer.writerow(["time_s", "ch0_input_V", "ch1_input_V"])
        f.flush()
        self._record_t0 = time.perf_counter()
        self.act_record_stop.setEnabled(True)
        self.act_record_start.setEnabled(False)
        self.status.showMessage(f"Recording input trace to {path}")

    def _stop_recording(self):
        was = self._record_file is not None
        if self._record_file is not None:
            try:
                self._record_file.close()
            except OSError:
                pass
        self._record_file = None
        self._record_writer = None
        self._record_t0 = None
        self.act_record_stop.setEnabled(False)
        self.act_record_start.setEnabled(True)
        if was:
            self.status.showMessage("Recording stopped")

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
        self._stop_recording()
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

"""
monitor_rp.py -- Qt control GUI for the redPitaLock two-channel PID stabilizer.

Connects to the Red Pitaya daemon over TCP and provides:
  * Per-channel PID enable/disable (off by default)
  * Live adjustment of Kp, Ki, Kd, setpoint, loop rate, error sign, AOM LUT
  * Real-time pyqtgraph plots of analog input and output voltage vs time
  * Recording of all telemetry (both channels) plus the complete board
    parameter set and every command sent, into an HDF5 file (Record bar / File menu)
  * Optional histogram of input with Gaussian overlay and mean / std (View menu)
  * Sample-and-hold: freeze the PID on an external TTL, an input threshold or
    manually; hold intervals are shaded on the input plot

Usage:
    python monitor_rp.py                       # default host rp-XXXX.local
    python monitor_rp.py 192.168.1.100         # explicit IP
    python monitor_rp.py rp-XXXX.local 5000    # explicit host + port
"""

import datetime
import socket
import os
import sys
import time
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer, QEventLoop, QSettings
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
    QScrollArea,
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
DEFAULT_HISTORY_SECONDS = 10.0   # rolling time window shown at start-up
MIN_HISTORY_SECONDS = 0.5
MAX_HISTORY_SECONDS = 600.0      # longest window the GUI keeps in memory
BUFFER_SAMPLES = int(MAX_HISTORY_SECONDS * 120)  # telemetry runs at ~100 Hz
SPCAL_DONE, SPCAL_FAILED, SPCAL_ABORT = 9, 10, 11   # firmware spcal_state values (config.h)


class Trace:
    """Append-only float buffer that hands out cheap numpy views.

    Keeps at most `capacity` samples. Appending is O(1); every `capacity`
    appends the newest half of the storage is moved to the front once, so the
    amortised cost stays constant and `view()` never copies.
    """

    def __init__(self, capacity: int):
        self.cap = int(capacity)
        self._buf = np.empty(2 * self.cap, dtype=float)
        self._n = 0

    def append(self, value: float):
        if self._n >= self._buf.size:
            self._buf[:self.cap] = self._buf[self.cap:]
            self._n = self.cap
        self._buf[self._n] = value
        self._n += 1

    def view(self) -> np.ndarray:
        """Newest `cap` samples at most, oldest first (a view, not a copy)."""
        start = max(0, self._n - self.cap)
        return self._buf[start:self._n]

    def clear(self):
        self._n = 0

    def __len__(self):
        return min(self._n, self.cap)


# ------------------------------------------------------------------ TCP worker


# ------------------------------------------------------------------ HDF5 recording

TELEM_COLUMNS = (
    ("host_time_s",  np.float64, "s", "PC wall-clock time of receipt (Unix epoch, time.time())"),
    ("board_time_s", np.float64, "s", "time stamp of the sample from the daemon"),
    ("input_V",      np.float64, "V", "low-pass-filtered analog input the PID acts on"),
    ("raw_V",        np.float64, "V", "unfiltered ADC sample of the same loop iteration"),
    ("target_out_V", np.float64, "V", "PID output before LUT / output limits"),
    ("actual_out_V", np.float64, "V", "voltage written to the DAC"),
    ("setpoint_V",   np.float64, "V", "setpoint"),
    ("enabled",      np.int8,    "",  "1 while the PID is on"),
    ("holding",      np.int8,    "",  "1 while the output is frozen by sample-and-hold"),
    ("gate",         np.int8,    "",  "1 while the hold gate is asserted"),
    ("level",        np.int8,    "",  "gate source level: 1 HIGH, 0 LOW, -1 no source"),
)


def _attr_value(v):
    """Store numbers as numbers, everything else as text."""
    if isinstance(v, (int, float, np.integer, np.floating)):
        return v
    s = str(v)
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


class H5Recorder:
    """Streams the telemetry of both channels into one HDF5 file.

    Layout of the file:
      /                attrs: format, host, port, note, start/stop time, duration,
                       GUI settings (history window etc.)
      /commands        compound dataset (t_s, command): every command the GUI sent
                       to the board while recording (setpoint changes, gains, ...)
      /ch0, /ch1       one resizable dataset per telemetry column (TELEM_COLUMNS,
                       each with 'unit' and 'description' attrs);
                       attrs 'params_start_<key>' / 'params_end_<key>' hold all
                       board parameters (the P line) at the start and at the end.

    Samples are buffered in memory and appended to the file by flush(), which
    the GUI calls once per second, so a crash loses at most one second.
    """

    CHUNK = 1024

    def __init__(self, path: str, host: str, port: int, note: str, n_ch: int = 2):
        import h5py  # imported lazily: only needed for recording
        self.path = path
        self.f = h5py.File(path, "w")
        self.t0_perf = time.perf_counter()
        self.t0_wall = time.time()
        self.n_samples = [0] * n_ch
        self.closed = False

        a = self.f.attrs
        a["format"] = "redPitaLock telemetry recording v1"
        a["gui_script"] = os.path.abspath(__file__)
        a["host"] = host
        a["port"] = int(port)
        a["note"] = note
        a["start_time_iso"] = datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")
        a["start_time_unix_s"] = self.t0_wall
        a["telemetry_rate_hz_nominal"] = 100.0

        self._ds: list[dict] = []
        self._buf: list[dict] = []
        for ch in range(n_ch):
            g = self.f.create_group(f"ch{ch}")
            g.attrs["channel_index"] = ch
            g.attrs["channel_name"] = f"Channel {ch + 1} (IN{ch + 1} -> OUT{ch + 1})"
            cols = {}
            for name, dtype, unit, desc in TELEM_COLUMNS:
                d = g.create_dataset(name, shape=(0,), maxshape=(None,), dtype=dtype,
                                     chunks=(self.CHUNK,))
                d.attrs["unit"] = unit
                d.attrs["description"] = desc
                cols[name] = d
            self._ds.append(cols)
            self._buf.append({name: [] for name, _, _, _ in TELEM_COLUMNS})

        self._cmd_dtype = np.dtype([("t_s", np.float64), ("command", h5py.string_dtype("utf-8"))])
        self._cmd_ds = self.f.create_dataset("commands", shape=(0,), maxshape=(None,),
                                             dtype=self._cmd_dtype, chunks=(256,))
        self._cmd_ds.attrs["description"] = (
            "commands sent to the board while recording; t_s = seconds since start_time_unix_s")
        self._cmd_buf: list[tuple] = []
        self.f.flush()

    # ..................................................... data in

    def elapsed_s(self) -> float:
        return time.perf_counter() - self.t0_perf

    def add_sample(self, ch, board_time_s, input_v, raw_v, target_out_v, actual_out_v,
                   setpoint_v, enabled, holding, gate, level):
        if self.closed or not (0 <= ch < len(self._buf)):
            return
        b = self._buf[ch]
        b["host_time_s"].append(time.time())
        b["board_time_s"].append(board_time_s)
        b["input_V"].append(input_v)
        b["raw_V"].append(input_v if raw_v is None else raw_v)
        b["target_out_V"].append(target_out_v)
        b["actual_out_V"].append(actual_out_v)
        b["setpoint_V"].append(setpoint_v)
        b["enabled"].append(int(enabled))
        b["holding"].append(int(holding))
        b["gate"].append(int(gate))
        b["level"].append(int(level))
        self.n_samples[ch] += 1

    def add_command(self, cmd: str):
        if not self.closed:
            self._cmd_buf.append((self.elapsed_s(), cmd))

    def set_params(self, ch: int, kv: dict, which: str):
        """Store a board parameter dict as attrs 'params_<which>_<key>' of /ch<ch>."""
        if self.closed or not (0 <= ch < len(self._ds)):
            return
        g = self.f[f"ch{ch}"]
        g.attrs[f"params_{which}_time_s"] = self.elapsed_s()
        for k, v in kv.items():
            g.attrs[f"params_{which}_{k}"] = _attr_value(v)

    def set_attr(self, key: str, value):
        if not self.closed:
            self.f.attrs[key] = _attr_value(value)

    # ..................................................... data out

    def flush(self):
        if self.closed:
            return
        for ch, b in enumerate(self._buf):
            n_new = len(b["host_time_s"])
            if n_new == 0:
                continue
            for name, lst in b.items():
                d = self._ds[ch][name]
                n0 = d.shape[0]
                d.resize((n0 + n_new,))
                d[n0:] = np.asarray(lst, dtype=d.dtype)
                lst.clear()
        if self._cmd_buf:
            arr = np.empty(len(self._cmd_buf), dtype=self._cmd_dtype)
            for i, (t, c) in enumerate(self._cmd_buf):
                arr[i] = (t, c)
            n0 = self._cmd_ds.shape[0]
            self._cmd_ds.resize((n0 + len(arr),))
            self._cmd_ds[n0:] = arr
            self._cmd_buf.clear()
        self.f.flush()

    def close(self):
        if self.closed:
            return
        try:
            self.flush()
            a = self.f.attrs
            a["stop_time_iso"] = datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")
            a["duration_s"] = self.elapsed_s()
            for ch, n in enumerate(self.n_samples):
                self.f[f"ch{ch}"].attrs["n_samples"] = n
            self.f.flush()
        finally:
            self.closed = True
            self.f.close()


class TcpWorker(QThread):
    """Background thread that maintains the TCP connection, parses telemetry,
    and forwards commands from the GUI."""

    # ch, time_s, in_v, target_out, actual_out, sp, enabled, holding, gate
    data_received = Signal(int, float, float, float, float, float, int, int, int, int, float)
    data_batch = Signal(list)       # every D sample of one recv, for the recorder: [(ch, ts, iv, tgt, act, sp, en, hold, gate, level, raw), ...]
    params_received = Signal(int, dict)                                  # ch, {key: value_str, ...}
    psd_received = Signal(int, int, float, object)                       # ch, n_bins, fs, np.array of PSD bins
    autotune_done = Signal(int, float, float, float, float, float)        # ch, Ku, Tu, Kp, Ki, Kd
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
        pending_psd_hdr = None

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
                data = self._sock.recv(262144)
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
            batch: list[tuple] = []
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
                            # Sample-and-hold fields (firmware >= this branch);
                            # older firmware omits them.
                            hold = int(parts[8]) if len(parts) > 8 else 0
                            gate = int(parts[9]) if len(parts) > 9 else 0
                            # Raw level of the gate source (TTL): 1 HIGH, 0 LOW,
                            # -1 when no gate source is selected.
                            level = int(parts[10]) if len(parts) > 10 else -1
                            # Unfiltered ADC sample (firmware >= this branch);
                            # older firmware: fall back to the filtered value.
                            raw = float(parts[11]) if len(parts) > 11 else iv
                            latest[ch] = (ts, iv, tgt, act, sp, en, hold, gate, level, raw)
                            batch.append((ch,) + latest[ch])
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
                    if len(parts) >= 7:
                        try:
                            ch = int(parts[1])
                            Ku = float(parts[2])
                            Tu = float(parts[3])
                            Kp = float(parts[4])
                            Ki = float(parts[5])
                            Kd = float(parts[6])
                            self.autotune_done.emit(ch, Ku, Tu, Kp, Ki, Kd)
                        except (ValueError, IndexError):
                            pass

            if batch:
                self.data_batch.emit(batch)
            if latest:
                for ch in sorted(latest.keys()):
                    ts, iv, tgt, act, sp, en, hold, gate, level, raw = latest[ch]
                    self.data_received.emit(ch, ts, iv, tgt, act, sp, en, hold, gate, level, raw)

        if self._sock:
            self._sock.close()


# ----------------------------------------------------------- Channel widget

class ChannelPanel(QWidget):
    """Controls + plots for a single PID channel."""

    command = Signal(str)  # emitted when the user changes a parameter
    window_changed = Signal(float)  # user zoomed the time axis -> new window (s)

    def __init__(self, ch_index: int, parent=None):
        super().__init__(parent)
        self.ch = ch_index
        self.input_data      = Trace(BUFFER_SAMPLES)
        self.target_out_data = Trace(BUFFER_SAMPLES)
        self.actual_out_data = Trace(BUFFER_SAMPLES)
        self.sp_data         = Trace(BUFFER_SAMPLES)
        self.t_data          = Trace(BUFFER_SAMPLES)  # server-side timestamp (s)
        self.hold_data       = Trace(BUFFER_SAMPLES)  # 1 while output held
        self.raw_data        = Trace(BUFFER_SAMPLES)  # unfiltered ADC sample
        self._raw_visible    = True
        self._t0             = None
        self._window_s       = DEFAULT_HISTORY_SECONDS  # rolling window length
        self._setting_range  = False  # True while we move the view ourselves
        self._view_pinned    = False  # True once the view follows the data
        self._autotuning     = False
        self._stats_visible  = False
        self._psd_visible    = False
        self._hold_regions: list[pg.LinearRegionItem] = []
        self._hold_status    = None   # (holding, gate) last shown

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
        self.sp_ki.setRange(0.0, 100000.0)
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

        # Integrator clamp: the integral state is limited to +/- this value.
        # Its contribution to the output is Ki * integral_max, so it must stay
        # above (out_max / Ki) or the loop can never reach the setpoint.
        ctrl_layout.addWidget(QLabel("Integral max"), row, 0)
        self.sp_integral_max = QDoubleSpinBox()
        self.sp_integral_max.setRange(0.0, 100.0)
        self.sp_integral_max.setDecimals(6)
        self.sp_integral_max.setSingleStep(0.01)
        self.sp_integral_max.setValue(0.5)
        self.sp_integral_max.setToolTip(
            "Anti-windup clamp on the integrator state (not in volts: the output "
            "contribution is Ki x this value). Keep it above out_max / Ki, otherwise the "
            "integrator cannot drive the output far enough and the input stops short of "
            "the setpoint. 0 switches the integral action off. Default 0.5.")
        self.sp_integral_max.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} integral_max {self.sp_integral_max.value():.6g}"))
        ctrl_layout.addWidget(self.sp_integral_max, row, 1)
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

        # --- Sample and hold ---
        self.hold_group = QGroupBox("Sample && Hold")
        hold_layout = QGridLayout()
        self.hold_group.setLayout(hold_layout)
        hrow = 0

        self.lbl_hold_status = QLabel("gate idle")
        self.lbl_hold_status.setAlignment(Qt.AlignCenter)
        self.lbl_hold_status.setStyleSheet(
            "QLabel { background-color: #e0e0e0; color: #333; padding: 4px; font-weight: bold; }")
        self.lbl_hold_status.setToolTip(
            "Gate: the hold source is asserted (also shown while PID is off, "
            "so you can check TTL wiring/polarity).\n"
            "HOLDING: PID is on and its output is frozen.")
        hold_layout.addWidget(self.lbl_hold_status, hrow, 0, 1, 2)
        hrow += 1

        # Virtual LED showing the raw level of the gate source (the TTL),
        # independent of the polarity setting.
        ttl_row = QWidget()
        ttl_layout = QHBoxLayout(ttl_row)
        ttl_layout.setContentsMargins(0, 0, 0, 0)
        ttl_layout.addWidget(QLabel("TTL level"))
        self.led_ttl = QLabel()
        self.led_ttl.setFixedSize(18, 18)
        self.lbl_ttl_level = QLabel("n/a")
        self.lbl_ttl_level.setToolTip(
            "Raw level of the selected gate source: the DIO pin, the input threshold "
            "comparator, or the TTL on the other analog input. "
            "Green = HIGH, dark = LOW, grey = no gate source selected.")
        ttl_layout.addStretch(1)
        ttl_layout.addWidget(self.led_ttl)
        ttl_layout.addWidget(self.lbl_ttl_level)
        hold_layout.addWidget(ttl_row, hrow, 0, 1, 2)
        hrow += 1
        self._set_ttl_led(-1)

        self.btn_hold = QPushButton("HOLD now")
        self.btn_hold.setCheckable(True)
        self.btn_hold.setStyleSheet(
            "QPushButton { padding: 6px; font-weight: bold; }"
            "QPushButton:checked { background-color: #FFB300; color: black; }")
        self.btn_hold.setToolTip("Manual hold: freezes the PID output immediately, in any mode.")
        self.btn_hold.toggled.connect(self._on_hold_toggled)
        hold_layout.addWidget(self.btn_hold, hrow, 0, 1, 2)
        hrow += 1

        hold_layout.addWidget(QLabel("Gate source"), hrow, 0)
        self.cb_hold_mode = QComboBox()
        other_in = f"IN{2 - self.ch}"
        self.cb_hold_mode.addItems(["Off", "External TTL (DIO)", "Input threshold",
                                    f"TTL on {other_in} (analog)"])
        self.cb_hold_mode.setToolTip(
            "Off: only the manual HOLD button.\n"
            "External TTL: freeze while DIO<n>_P on E1 is at the hold level "
            "(3.3 V logic, NOT 5 V tolerant).\n"
            "Input threshold: freeze while the raw photodiode voltage is below the threshold.\n"
            f"TTL on {other_in}: the TTL goes into the other SMA input (5 V is safe in LV mode, "
            "reads as +1 V). HIGH = above threshold + hysteresis, LOW = below threshold; "
            "'Hold while pin' selects the level.")
        self.cb_hold_mode.currentIndexChanged.connect(self._on_hold_mode_changed)
        hold_layout.addWidget(self.cb_hold_mode, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("DIO pin (E1)"), hrow, 0)
        self.sp_hold_pin = QSpinBox()
        self.sp_hold_pin.setRange(0, 7)
        self.sp_hold_pin.setValue(self.ch)
        self.sp_hold_pin.setToolTip("n selects DIO<n>_P. DIO0_P = E1 pin 3, DIO1_P = pin 5, ... GND = pins 25/26.")
        self.sp_hold_pin.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_pin {self.sp_hold_pin.value()}"))
        hold_layout.addWidget(self.sp_hold_pin, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Hold while pin/input"), hrow, 0)
        self.cb_hold_pol = QComboBox()
        self.cb_hold_pol.addItems(["LOW", "HIGH"])
        self.cb_hold_pol.currentIndexChanged.connect(
            lambda idx: self.command.emit(f"SET {self.ch} hold_pol {idx}"))
        hold_layout.addWidget(self.cb_hold_pol, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Threshold (V)"), hrow, 0)
        self.sp_hold_thresh = QDoubleSpinBox()
        self.sp_hold_thresh.setRange(-1.0, 1.0)
        self.sp_hold_thresh.setDecimals(4)
        self.sp_hold_thresh.setSingleStep(0.01)
        self.sp_hold_thresh.setValue(0.05)
        self.sp_hold_thresh.setToolTip(
            "Threshold mode: hold while the raw input is below this voltage.\n"
            "TTL on analog input: level that separates LOW from HIGH (0.5 V for a TTL).")
        self.sp_hold_thresh.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_thresh {self.sp_hold_thresh.value():.4f}"))
        hold_layout.addWidget(self.sp_hold_thresh, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Hysteresis (V)"), hrow, 0)
        self.sp_hold_hyst = QDoubleSpinBox()
        self.sp_hold_hyst.setRange(0.0, 1.0)
        self.sp_hold_hyst.setDecimals(4)
        self.sp_hold_hyst.setSingleStep(0.005)
        self.sp_hold_hyst.setValue(0.01)
        self.sp_hold_hyst.setToolTip("Threshold mode: release only once the input exceeds threshold + hysteresis.")
        self.sp_hold_hyst.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_hyst {self.sp_hold_hyst.value():.4f}"))
        hold_layout.addWidget(self.sp_hold_hyst, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Debounce (samples)"), hrow, 0)
        self.sp_hold_debounce = QSpinBox()
        self.sp_hold_debounce.setRange(1, 100000)
        self.sp_hold_debounce.setValue(2)
        self.sp_hold_debounce.setToolTip("Consecutive loop iterations the gate must be asserted before holding.")
        self.sp_hold_debounce.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_debounce {self.sp_hold_debounce.value()}"))
        hold_layout.addWidget(self.sp_hold_debounce, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Resume delay (us)"), hrow, 0)
        self.sp_resume_delay = QSpinBox()
        self.sp_resume_delay.setRange(0, 10000000)
        self.sp_resume_delay.setSingleStep(100)
        self.sp_resume_delay.setValue(200)
        self.sp_resume_delay.setToolTip(
            "Keep holding this long after the gate releases, so AOM/RF/photodiode can settle "
            "before the loop re-engages (rounded up to whole loop periods).")
        self.sp_resume_delay.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} resume_delay {self.sp_resume_delay.value()}"))
        hold_layout.addWidget(self.sp_resume_delay, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Hold lookback (ms)"), hrow, 0)
        self.sp_hold_lookback = QSpinBox()
        self.sp_hold_lookback.setRange(0, 60000)
        self.sp_hold_lookback.setSingleStep(10)
        self.sp_hold_lookback.setValue(0)
        self.sp_hold_lookback.setToolTip(
            "Hold the output the loop had this long BEFORE the gate asserted, instead of the\n"
            "last output. Use it when the TTL arrives after the beam is already gone, so the\n"
            "controller's reaction to the dark photodiode is not frozen into the held value.")
        self.sp_hold_lookback.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_lookback {self.sp_hold_lookback.value()}"))
        hold_layout.addWidget(self.sp_hold_lookback, hrow, 1)
        hrow += 1

        hold_layout.addWidget(QLabel("Hold average (ms)"), hrow, 0)
        self.sp_hold_avg = QSpinBox()
        self.sp_hold_avg.setRange(0, 60000)
        self.sp_hold_avg.setSingleStep(10)
        self.sp_hold_avg.setValue(0)
        self.sp_hold_avg.setToolTip(
            "Average the held output over a window of this length ending at the lookback\n"
            "point (0 = single sample). Smooths noise out of the held value.")
        self.sp_hold_avg.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_avg {self.sp_hold_avg.value()}"))
        hold_layout.addWidget(self.sp_hold_avg, hrow, 1)
        hrow += 1

        self.chk_hold_park = QCheckBox("Park output while holding")
        self.chk_hold_park.setToolTip(
            "Unchecked: keep the last PID output on the DAC during the hold (default, right when "
            "an upstream TTL switch blanks the beam).\nChecked: drive the park voltage instead.")
        self.chk_hold_park.toggled.connect(self._on_hold_park_toggled)
        hold_layout.addWidget(self.chk_hold_park, hrow, 0, 1, 2)
        hrow += 1

        hold_layout.addWidget(QLabel("Park voltage (V)"), hrow, 0)
        self.sp_hold_park_v = QDoubleSpinBox()
        self.sp_hold_park_v.setRange(-1.0, 1.0)
        self.sp_hold_park_v.setDecimals(4)
        self.sp_hold_park_v.setSingleStep(0.01)
        self.sp_hold_park_v.setValue(0.0)
        self.sp_hold_park_v.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} hold_park_v {self.sp_hold_park_v.value():.4f}"))
        hold_layout.addWidget(self.sp_hold_park_v, hrow, 1)
        hrow += 1

        ctrl_layout.addWidget(self.hold_group, row, 0, 1, 2)
        row += 1
        self._update_hold_fields_visibility()

        # --- External setpoint (control voltage on the other fast input) ---
        other = 2 - self.ch
        self.ext_group = QGroupBox(f"External setpoint (IN{other})")
        ext_layout = QGridLayout()
        self.ext_group.setLayout(ext_layout)
        erow = 0

        ext_layout.addWidget(QLabel("Setpoint source"), erow, 0)
        self.cb_sp_source = QComboBox()
        self.cb_sp_source.addItems(["Internal (Setpoint field)", f"External: IN{other} x max"])
        self.cb_sp_source.setToolTip(
            f"External: the setpoint follows the control voltage on IN{other}:\n"
            f"setpoint = min + (max - min) x (IN{other} / full scale), clamped to 0..1.\n"
            "The Setpoint field is then read-only and shows the live value.")
        self.cb_sp_source.currentIndexChanged.connect(self._on_sp_source_changed)
        ext_layout.addWidget(self.cb_sp_source, erow, 1)
        erow += 1

        ext_layout.addWidget(QLabel("Max setpoint (% of range)"), erow, 0)
        self.sp_ext_pct = QDoubleSpinBox()
        self.sp_ext_pct.setRange(0.0, 100.0)
        self.sp_ext_pct.setDecimals(1)
        self.sp_ext_pct.setSingleStep(1.0)
        self.sp_ext_pct.setValue(90.0)
        self.sp_ext_pct.setSuffix(" %")
        self.sp_ext_pct.setToolTip(
            "Max setpoint = zero + percentage x (full - zero), where zero and full are the\n"
            "inputs measured at minimum and maximum output (\"Measure laser range\").\n"
            "Changing it rescales the max setpoint from the last measurement.")
        self.sp_ext_pct.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} sp_ext_pct {self.sp_ext_pct.value():.1f}"))
        ext_layout.addWidget(self.sp_ext_pct, erow, 1)
        erow += 1

        self.btn_spcal = QPushButton("Measure laser range")
        self.btn_spcal.setToolTip(
            "Ramps the output to 'PID out min', averages the input (zero), ramps to\n"
            "'PID out max', averages again (full), sets min setpoint = zero and\n"
            "max setpoint = zero + percentage x (full - zero), ramps back and resumes\n"
            "the PID. Run it while the beam is on (TTL high). Press again to abort.")
        self.btn_spcal.setStyleSheet(
            "QPushButton { background-color: #6A1B9A; color: white; font-weight: bold; padding: 6px; }")
        self.btn_spcal.clicked.connect(self._on_spcal_clicked)
        ext_layout.addWidget(self.btn_spcal, erow, 0, 1, 2)
        erow += 1

        self.lbl_spcal = QLabel("not measured yet")
        self.lbl_spcal.setWordWrap(True)
        self.lbl_spcal.setStyleSheet("QLabel { color: #555; }")
        ext_layout.addWidget(self.lbl_spcal, erow, 0, 1, 2)
        erow += 1

        ext_layout.addWidget(QLabel("Min setpoint (V)"), erow, 0)
        self.sp_ext_min = QDoubleSpinBox()
        self.sp_ext_min.setRange(-1.0, 1.0)
        self.sp_ext_min.setDecimals(4)
        self.sp_ext_min.setSingleStep(0.001)
        self.sp_ext_min.setValue(0.0)
        self.sp_ext_min.setToolTip("Setpoint at zero control voltage. May be negative. Set to the input measured at "
                                   "minimum output, or type a value directly.")
        self.sp_ext_min.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} sp_ext_min {self.sp_ext_min.value():.4f}"))
        ext_layout.addWidget(self.sp_ext_min, erow, 1)
        erow += 1

        ext_layout.addWidget(QLabel("Max setpoint (V)"), erow, 0)
        self.sp_ext_max = QDoubleSpinBox()
        self.sp_ext_max.setRange(-1.0, 1.0)
        self.sp_ext_max.setDecimals(4)
        self.sp_ext_max.setSingleStep(0.001)
        self.sp_ext_max.setValue(0.0)
        self.sp_ext_max.setToolTip("Setpoint at full control voltage. Set by the measurement, "
                                   "or type a value directly.")
        self.sp_ext_max.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} sp_ext_max {self.sp_ext_max.value():.4f}"))
        ext_layout.addWidget(self.sp_ext_max, erow, 1)
        erow += 1

        ext_layout.addWidget(QLabel("Control full scale (V)"), erow, 0)
        self.sp_ext_ref = QDoubleSpinBox()
        self.sp_ext_ref.setRange(0.01, 1.0)
        self.sp_ext_ref.setDecimals(3)
        self.sp_ext_ref.setSingleStep(0.1)
        self.sp_ext_ref.setValue(1.0)
        self.sp_ext_ref.setToolTip(f"Control voltage on IN{other} that corresponds to the max setpoint "
                                   "(1 V for a 0..1 V control signal).")
        self.sp_ext_ref.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} sp_ext_ref {self.sp_ext_ref.value():.4f}"))
        ext_layout.addWidget(self.sp_ext_ref, erow, 1)
        erow += 1

        ext_layout.addWidget(QLabel("Ramp / average (ms)"), erow, 0)
        ramp_box = QHBoxLayout()
        self.sp_spcal_ramp = QSpinBox()
        self.sp_spcal_ramp.setRange(1, 10000)
        self.sp_spcal_ramp.setValue(200)
        self.sp_spcal_ramp.setToolTip("Output ramp time up and down during the measurement (ms)")
        self.sp_spcal_ramp.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} spcal_ramp_ms {self.sp_spcal_ramp.value()}"))
        ramp_box.addWidget(self.sp_spcal_ramp)
        self.sp_spcal_avg = QSpinBox()
        self.sp_spcal_avg.setRange(1, 10000)
        self.sp_spcal_avg.setValue(100)
        self.sp_spcal_avg.setToolTip("Settle time, then averaging time at full output (ms each)")
        self.sp_spcal_avg.editingFinished.connect(
            lambda: self.command.emit(f"SET {self.ch} spcal_avg_ms {self.sp_spcal_avg.value()}"))
        ramp_box.addWidget(self.sp_spcal_avg)
        ext_layout.addLayout(ramp_box, erow, 1)
        erow += 1

        self.lbl_ext_live = QLabel("")
        self.lbl_ext_live.setStyleSheet("QLabel { font-family: monospace; }")
        ext_layout.addWidget(self.lbl_ext_live, erow, 0, 1, 2)
        erow += 1

        ctrl_layout.addWidget(self.ext_group, row, 0, 1, 2)
        row += 1
        self._ctrl_v: Optional[float] = None
        self._ext_live_text = None
        self._spcal_timer = QTimer()
        self._spcal_timer.setInterval(300)
        self._spcal_timer.timeout.connect(self._spcal_poll)
        self._spcal_started: Optional[float] = None
        self._update_sp_source_fields()

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

        # The control column is taller than most screens now: scroll it.
        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidget(ctrl_box)
        ctrl_scroll.setWidgetResizable(True)
        ctrl_scroll.setFrameShape(QScrollArea.NoFrame)
        ctrl_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        ctrl_scroll.setFixedWidth(300)

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
        # Unfiltered ADC samples (one per telemetry tick), light grey under
        # the filtered trace; View > Show raw ADC samples toggles it.
        self.curve_raw   = self.plot_input.plot(pen=pg.mkPen("#eceff1", width=1), name="Raw ADC")
        self.curve_raw.setZValue(-1)          # under the filtered trace
        self.curve_raw.setVisible(True)

        # Right-hand axis: the same vertical range expressed as deviation from
        # the setpoint in percent, (V - setpoint) / |setpoint| * 100. It is a
        # pure relabelling of the left axis, so it follows every zoom.
        # Mean / std of the samples currently visible, as text in the top-right
        # corner of the input plot (parented to the ViewBox: pixel coordinates).
        self.lbl_window_stats = pg.TextItem(anchor=(1, 0), color="#eeeeee",
                                            fill=pg.mkBrush(0, 0, 0, 150))
        self.lbl_window_stats.setZValue(100)
        self.lbl_window_stats.setParentItem(self.plot_input.getViewBox())
        self._window_stats_state = None

        self.plot_input.showAxis("right")
        self._dev_axis = self.plot_input.getAxis("right")
        self._dev_axis.unlinkFromView()
        self._dev_axis.setLabel("Deviation from setpoint", units="%")
        self._dev_axis.enableAutoSIPrefix(False)
        self._dev_axis_state = None   # (y0, y1, setpoint) last applied
        self.plot_input.getViewBox().sigYRangeChanged.connect(
            lambda *_: self._update_deviation_axis())
        # (downsampling/clipToView disabled — 400 points is cheap to render)

        self.plot_output = pg.PlotWidget(title=f"Ch{self.ch+1} Output Voltage")
        self.plot_output.setLabel("left", "Voltage", units="V")
        self.plot_output.setLabel("bottom", "Time", units="s")
        self.plot_output.addLegend(offset=(10, 10))
        self.curve_target_out = self.plot_output.plot(pen=pg.mkPen("#FF5722", width=2, style=Qt.DashLine), name="Target")
        self.curve_actual_out = self.plot_output.plot(pen=pg.mkPen("#E91E63", width=2), name="Actual")

        # Strip-chart behaviour: the time axis always ends at the newest sample
        # and spans the rolling window. Mouse wheel / drag over the plot body
        # only changes the window length (see _on_view_x_range_changed); the
        # voltage axis keeps auto-ranging on the visible data. Zoom the voltage
        # axis by scrolling or dragging on the axis itself; the small "A"
        # button restores auto-range.
        self.plot_output.setXLink(self.plot_input)
        for pw in (self.plot_input, self.plot_output):
            vb = pw.getViewBox()
            vb.setMouseEnabled(x=True, y=False)
            vb.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
            vb.setAutoVisible(y=True)
            vb.sigXRangeChanged.connect(self._on_view_x_range_changed)

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

        self.lbl_psd_metrics = QLabel()
        self.lbl_psd_metrics.setFont(QFont("Consolas, Courier, monospace", 9))
        self.lbl_psd_metrics.setTextFormat(Qt.PlainText)
        self.lbl_psd_metrics.setWordWrap(True)
        self.lbl_psd_metrics.hide()

        plot_layout.addWidget(self.plot_input)
        plot_layout.addWidget(self.plot_output)
        plot_layout.addWidget(self.plot_stats)
        plot_layout.addWidget(self.plot_psd)
        plot_layout.addWidget(self.lbl_psd_metrics)

        root.addWidget(ctrl_scroll)
        root.addWidget(plot_widget, stretch=1)

    # ...................................................... param sync

    def sync_from_params(self, kv: dict):
        """Update all widgets from a key=value dict received from the server.
        Blocks signals so we don't re-emit SET commands back to the device."""
        def _f(v): return float(v)
        def _i(v): return int(float(v))

        synced = (
            self.btn_enable, self.sp_setpoint, self.sp_out_min, self.sp_out_max,
            self.sp_kp, self.sp_ki, self.sp_kd, self.sp_loop, self.cb_sign,
            self.chk_lut, self.cb_out_mode, self.sp_manual_v, self.sp_wave_freq,
            self.sp_wave_amp, self.sp_wave_offset, self.sp_at_amp,
            self.btn_hold, self.cb_hold_mode, self.sp_hold_pin, self.cb_hold_pol,
            self.sp_hold_thresh, self.sp_hold_hyst, self.sp_hold_debounce,
            self.sp_resume_delay, self.chk_hold_park, self.sp_hold_park_v,
            self.sp_hold_lookback, self.sp_hold_avg,
            self.cb_sp_source, self.sp_ext_pct, self.sp_ext_min, self.sp_ext_max, self.sp_ext_ref,
            self.sp_spcal_ramp, self.sp_spcal_avg,
        )
        for widget in synced:
            widget.blockSignals(True)

        if "enabled" in kv:
            en = _i(kv["enabled"])
            self.btn_enable.setChecked(bool(en))
            self.btn_enable.setText("PID ON" if en else "PID OFF")
        if "sp_user" in kv:
            # The user's own setpoint; "setpoint" is the live value, which in
            # external mode follows the control voltage.
            self.sp_setpoint.setValue(_f(kv["sp_user"]))
        elif "setpoint" in kv:
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
            self.sp_integral_max.setValue(_f(kv["integral_max"]))
        # Sample and hold
        if "hold" in kv:
            self.btn_hold.setChecked(bool(_i(kv["hold"])))
            self._refresh_hold_button_text()
        if "hold_mode" in kv:
            self.cb_hold_mode.setCurrentIndex(max(0, min(3, _i(kv["hold_mode"]))))
        if "hold_pin" in kv:
            self.sp_hold_pin.setValue(_i(kv["hold_pin"]))
        if "hold_pol" in kv:
            self.cb_hold_pol.setCurrentIndex(1 if _i(kv["hold_pol"]) else 0)
        if "hold_thresh" in kv:
            self.sp_hold_thresh.setValue(_f(kv["hold_thresh"]))
        if "hold_hyst" in kv:
            self.sp_hold_hyst.setValue(_f(kv["hold_hyst"]))
        if "hold_debounce" in kv:
            self.sp_hold_debounce.setValue(_i(kv["hold_debounce"]))
        if "resume_delay" in kv:
            self.sp_resume_delay.setValue(_i(kv["resume_delay"]))
        if "hold_lookback" in kv:
            self.sp_hold_lookback.setValue(_i(kv["hold_lookback"]))
        if "hold_avg" in kv:
            self.sp_hold_avg.setValue(_i(kv["hold_avg"]))
        if "hold_park" in kv:
            self.chk_hold_park.setChecked(bool(_i(kv["hold_park"])))
        if "hold_park_v" in kv:
            self.sp_hold_park_v.setValue(_f(kv["hold_park_v"]))
        # External setpoint
        if "sp_source" in kv:
            self.cb_sp_source.setCurrentIndex(1 if _i(kv["sp_source"]) else 0)
        if "sp_ext_pct" in kv:
            self.sp_ext_pct.setValue(_f(kv["sp_ext_pct"]))
        if "sp_ext_min" in kv:
            self.sp_ext_min.setValue(_f(kv["sp_ext_min"]))
        if "sp_ext_max" in kv:
            self.sp_ext_max.setValue(_f(kv["sp_ext_max"]))
        if "sp_ext_ref" in kv:
            self.sp_ext_ref.setValue(_f(kv["sp_ext_ref"]))
        if "spcal_ramp_ms" in kv:
            self.sp_spcal_ramp.setValue(_i(kv["spcal_ramp_ms"]))
        if "spcal_avg_ms" in kv:
            self.sp_spcal_avg.setValue(_i(kv["spcal_avg_ms"]))

        for widget in synced:
            widget.blockSignals(False)

        self._update_manual_fields_visibility()
        self._update_hold_fields_visibility()
        self._update_sp_source_fields()
        if "spcal_state" in kv:
            st = _i(kv["spcal_state"])
            if self._spcal_started is not None and st in (0, SPCAL_DONE, SPCAL_FAILED):
                self._spcal_finish(kv)
            else:
                self._show_spcal_result(kv)

    # ...................................................... callbacks


    # ...................................................... external setpoint

    def _on_sp_source_changed(self, idx):
        self.command.emit(f"SET {self.ch} sp_source {idx}")
        self._update_sp_source_fields()

    def _update_sp_source_fields(self):
        ext = self.cb_sp_source.currentIndex() == 1
        self.sp_setpoint.setEnabled(not ext)
        self.sp_setpoint.setToolTip(
            "Read-only while the setpoint follows the external control voltage."
            if ext else
            "Target input voltage at the fast ADC (LV SMA ~ +/-1 V). "
            "Values outside that range cannot be reached.")
        if not ext:
            self.lbl_ext_live.setText("")
            self._ext_live_text = None

    def set_control_voltage(self, v: float):
        """Latest input of the other channel (= the control voltage of this one)."""
        self._ctrl_v = v

    def _refresh_ext_live(self):
        if self.cb_sp_source.currentIndex() != 1:
            return
        other = 2 - self.ch
        sp = float(self.sp_data.view()[-1]) if len(self.sp_data) else float("nan")
        ref = self.sp_ext_ref.value()
        lo, hi = self.sp_ext_min.value(), self.sp_ext_max.value()
        if self._ctrl_v is None:
            text = f"IN{other}: n/a   setpoint {1e3 * sp:.2f} mV"
        else:
            frac = min(1.0, max(0.0, self._ctrl_v / ref)) if ref > 0 else 0.0
            text = (f"IN{other} {self._ctrl_v:.3f} V ({100 * frac:.0f} %)  "
                    f"-> setpoint {1e3 * sp:.2f} mV  [{1e3 * lo:.1f} .. {1e3 * hi:.1f}]")
        if text != self._ext_live_text:
            self._ext_live_text = text
            self.lbl_ext_live.setText(text)

    def _on_spcal_clicked(self):
        if self._spcal_started is not None:
            self.command.emit(f"SET {self.ch} spcal 0")
            self.lbl_spcal.setText("aborting ...")
            return
        self.command.emit(f"SET {self.ch} spcal 1")
        self._spcal_started = time.perf_counter()
        self.btn_spcal.setText("Measuring ... (click to abort)")
        self.lbl_spcal.setText("ramping the output to PID out min ...")
        self._spcal_timer.start()

    def _spcal_poll(self):
        if self._spcal_started is None:
            self._spcal_timer.stop()
            return
        if time.perf_counter() - self._spcal_started > 60.0:
            self._spcal_finish(None)
            self.lbl_spcal.setText("no result from the board (timeout)")
            return
        self.command.emit(f"GET {self.ch} params")

    def _spcal_finish(self, kv):
        self._spcal_timer.stop()
        self._spcal_started = None
        self.btn_spcal.setText("Measure laser range")
        if kv is not None:
            self._show_spcal_result(kv)

    def _show_spcal_result(self, kv):
        try:
            st = int(float(kv.get("spcal_state", 0)))
            zero = float(kv.get("spcal_zero", 0.0))
            full = float(kv.get("spcal_full", 0.0))
            out_lo = float(kv.get("spcal_out_min", 0.0))
            out = float(kv.get("spcal_out", 0.0))
            mn = float(kv.get("sp_ext_min", 0.0))
            mx = float(kv.get("sp_ext_max", 0.0))
            pct = float(kv.get("sp_ext_pct", 0.0))
        except ValueError:
            return
        if st == SPCAL_FAILED:
            self.lbl_spcal.setText(
                f"FAILED: input {1e3 * zero:.2f} mV at {out_lo:.2f} V output, "
                f"{1e3 * full:.2f} mV at {out:.2f} V (no response, beam off or aborted). "
                "Setpoint range unchanged.")
            self.lbl_spcal.setStyleSheet("QLabel { color: #b71c1c; }")
        elif st == SPCAL_DONE or (st == 0 and full > zero):
            self.lbl_spcal.setText(
                f"input {1e3 * zero:.2f} mV at {out_lo:.2f} V, {1e3 * full:.2f} mV at {out:.2f} V "
                f"output -> setpoint range {1e3 * mn:.2f} .. {1e3 * mx:.2f} mV ({pct:.0f} %)")
            self.lbl_spcal.setStyleSheet("QLabel { color: #1b5e20; }")
        elif 1 <= st <= 8 or st == SPCAL_ABORT:
            self.lbl_spcal.setText("measuring ..." if st < 5 else "measuring at maximum output ...")
        else:
            self.lbl_spcal.setText("not measured yet")
            self.lbl_spcal.setStyleSheet("QLabel { color: #555; }")

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

    # ...................................................... sample & hold

    def _refresh_hold_button_text(self):
        self.btn_hold.setText("Release HOLD" if self.btn_hold.isChecked() else "HOLD now")

    def _on_hold_toggled(self, checked):
        self._refresh_hold_button_text()
        self.command.emit(f"SET {self.ch} hold {int(checked)}")

    def _on_hold_mode_changed(self, idx):
        if idx == 3 and self.sp_hold_thresh.value() < 0.2:
            # Threshold still at the photodiode-level default: preset it for
            # a TTL that clips at +1 V on the analog input.
            self.sp_hold_thresh.setValue(0.5)
            self.sp_hold_hyst.setValue(0.1)
            self.command.emit(f"SET {self.ch} hold_thresh 0.5000")
            self.command.emit(f"SET {self.ch} hold_hyst 0.1000")
        self.command.emit(f"SET {self.ch} hold_mode {idx}")
        self._update_hold_fields_visibility()

    def _on_hold_park_toggled(self, checked):
        self.command.emit(f"SET {self.ch} hold_park {int(checked)}")
        self._update_hold_fields_visibility()

    def _update_hold_fields_visibility(self):
        mode = self.cb_hold_mode.currentIndex()
        dio = mode == 1
        thresh = mode == 2
        analog = mode == 3
        self.sp_hold_pin.setEnabled(dio)
        self.cb_hold_pol.setEnabled(dio or analog)
        self.sp_hold_thresh.setEnabled(thresh or analog)
        self.sp_hold_hyst.setEnabled(thresh or analog)
        self.sp_hold_debounce.setEnabled(dio or thresh or analog)
        self.sp_hold_lookback.setEnabled(dio or thresh or analog)
        self.sp_hold_avg.setEnabled(dio or thresh or analog)
        self.sp_hold_park_v.setEnabled(self.chk_hold_park.isChecked())

    def _set_ttl_led(self, level: int):
        if level > 0:
            color, text = "#2ecc40", "HIGH"
        elif level == 0:
            color, text = "#2b3a2b", "LOW"
        else:
            color, text = "#c0c0c0", "n/a"
        self.led_ttl.setStyleSheet(
            f"QLabel {{ background-color: {color}; border: 1px solid #555; border-radius: 9px; }}")
        self.lbl_ttl_level.setText(text)

    def _update_hold_status(self, holding: int, gate: int, level: int = -1):
        """Update the status lamp and TTL LED; cheap when nothing changed."""
        if self._hold_status == (holding, gate, level):
            return
        prev = self._hold_status
        self._hold_status = (holding, gate, level)
        if prev is None or prev[2] != level:
            self._set_ttl_led(level)
        if prev is not None and prev[:2] == (holding, gate):
            return
        if holding:
            self.lbl_hold_status.setText("HOLDING")
            style = "background-color: #FFB300; color: black;"
        elif gate:
            self.lbl_hold_status.setText("gate asserted (PID off)")
            style = "background-color: #FFE082; color: black;"
        else:
            self.lbl_hold_status.setText("gate idle")
            style = "background-color: #e0e0e0; color: #333;"
        self.lbl_hold_status.setStyleSheet(
            "QLabel { " + style + " padding: 4px; font-weight: bold; }")

    def _refresh_hold_regions(self, x: np.ndarray, hold: np.ndarray):
        """Shade the intervals of the input plot during which the output was held."""
        hold = hold.astype(int)
        if hold.size != x.size or hold.size == 0:
            intervals = []
        else:
            padded = np.concatenate(([0], hold, [0]))
            edges = np.diff(padded)
            starts = np.flatnonzero(edges == 1)
            ends = np.flatnonzero(edges == -1) - 1
            intervals = [(x[s], x[e]) for s, e in zip(starts, ends)]

        # Reuse region items; create more only when needed.
        while len(self._hold_regions) < len(intervals):
            region = pg.LinearRegionItem(
                values=(0, 0), movable=False,
                brush=pg.mkBrush(255, 179, 0, 60), pen=pg.mkPen(None))
            region.setZValue(-10)
            self.plot_input.addItem(region)
            self._hold_regions.append(region)
        for region, (a, b) in zip(self._hold_regions, intervals):
            region.setRegion((a, b))
            region.show()
        for region in self._hold_regions[len(intervals):]:
            region.hide()

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

    @Slot(float, float, float, float, float)
    def on_autotune_done(self, Ku, Tu, Kp, Ki, Kd):
        self._autotuning = False
        for w in (self.sp_kp, self.sp_ki, self.sp_kd):
            w.blockSignals(True)
        self.sp_kp.setValue(Kp)
        self.sp_ki.setValue(Ki)
        self.sp_kd.setValue(Kd)
        for w in (self.sp_kp, self.sp_ki, self.sp_kd):
            w.blockSignals(False)
        self.btn_autotune.setText("Autotune")

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

    def set_window_seconds(self, seconds: float):
        """Set the length of the rolling time window shown in the plots."""
        self._window_s = float(np.clip(seconds, MIN_HISTORY_SECONDS, MAX_HISTORY_SECONDS))

    def window_seconds(self) -> float:
        return self._window_s

    def _on_view_x_range_changed(self, _vb, rng):
        """User zoomed/panned the time axis: adopt the new span as window length.

        The view is pinned back to the newest sample on the next refresh, so a
        zoom never freezes the axis -- it only makes the rolling window longer
        or shorter. Pans change nothing (the span stays the same).
        """
        if self._setting_range or not self._view_pinned:
            # Ignore our own updates and the auto-range events that occur
            # before the first sample (they do not come from the user).
            return
        span = float(rng[1] - rng[0])
        span = float(np.clip(span, MIN_HISTORY_SECONDS, MAX_HISTORY_SECONDS))
        if abs(span - self._window_s) > 1e-9:
            self._window_s = span
            self.window_changed.emit(span)

    def reset_timebase(self):
        """Clear history and restart the time axis (e.g. on new TCP connection)."""
        self.input_data.clear()
        self.target_out_data.clear()
        self.actual_out_data.clear()
        self.sp_data.clear()
        self.t_data.clear()
        self.hold_data.clear()
        self.raw_data.clear()
        self._t0 = None
        self._view_pinned = False

    def set_statistics_visible(self, visible: bool):
        self._stats_visible = visible
        self.plot_stats.setVisible(visible)
        if visible:
            self._refresh_stats_plot()

    def set_raw_visible(self, visible: bool):
        self._raw_visible = visible
        self.curve_raw.setVisible(visible)
        if not visible:
            self.curve_raw.setData([], [])

    def set_psd_visible(self, visible: bool):
        self._psd_visible = visible
        self.plot_psd.setVisible(visible)
        self.lbl_psd_metrics.setVisible(visible)

    @Slot(float, float, float, float, float, int, int, int, int, float)
    def add_sample(self, time_s, input_v, target_out_v, actual_out_v, setpoint_v,
                   enabled, holding=0, gate=0, level=-1, raw_v=None):
        if self._t0 is None:
            self._t0 = time_s
        self.t_data.append(time_s - self._t0)
        self.input_data.append(input_v)
        self.target_out_data.append(target_out_v)
        self.actual_out_data.append(actual_out_v)
        self.sp_data.append(setpoint_v)
        self.hold_data.append(1 if holding else 0)
        self.raw_data.append(input_v if raw_v is None else raw_v)
        self._update_hold_status(holding, gate, level)

    def set_psd_data(self, n_bins: int, fs: float, bins):
        if not self._psd_visible:
            return
        freqs = np.arange(n_bins) * (fs / ((n_bins - 1) * 2))
        mask = freqs > 0
        self.curve_psd.setData(freqs[mask], bins[mask])
        bw = fs / 2.0
        if bw >= 1e6:
            bw_str = f"{bw / 1e6:.1f} MHz"
        elif bw >= 1e3:
            bw_str = f"{bw / 1e3:.1f} kHz"
        else:
            bw_str = f"{bw:.0f} Hz"

        df = fs / ((n_bins - 1) * 2)
        sigma2 = float(np.sum(bins[1:]) * df)
        sigma = np.sqrt(sigma2)
        v_mean = self.sp_setpoint.value()

        if sigma >= 1e-3:
            sigma_str = f"{sigma * 1e3:.3f} mV"
        else:
            sigma_str = f"{sigma * 1e6:.2f} uV"

        if abs(v_mean) > 1e-9:
            frac = sigma / abs(v_mean)
            gate_err = (np.pi ** 2 / 4.0) * sigma2 / (v_mean ** 2)
            title = (f"Ch{self.ch+1} PSD  (BW {bw_str})  "
                     f"gate error: {gate_err:.2e}")
            metrics = (
                f"RMS noise: {sigma_str}   "
                f"fractional: {frac:.2e}   "
                f"pi-pulse 1-F: {gate_err:.2e}"
            )
        else:
            title = f"Ch{self.ch+1} PSD  (BW {bw_str})  setpoint=0, no gate error"
            metrics = f"RMS noise: {sigma_str}   (set nonzero setpoint for gate error)"

        self.plot_psd.setTitle(title)
        self.lbl_psd_metrics.setText(metrics)

    def _refresh_stats_plot(self, data: Optional[np.ndarray] = None):
        if data is None:
            data = self._visible_slice()[1]
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

    def _refresh_window_stats(self, inp: np.ndarray, hold: np.ndarray, raw: np.ndarray):
        """Mean and std of the visible samples (held samples excluded).

        Two std values: of the filtered input the PID sees (what the plot
        shows) and of the unfiltered ADC samples (all noise up to the loop
        rate; the number the optimizers and the sweep report).
        """
        vb = self.plot_input.getViewBox()
        self.lbl_window_stats.setPos(vb.width() - 6, 6)
        if inp.size == 0:
            return
        keep = hold.astype(int) == 0 if hold.size == inp.size else np.ones(inp.size, dtype=bool)
        data = inp[keep]
        rawd = raw[keep] if raw.size == inp.size else data
        n_excl = int(inp.size - data.size)
        if data.size < 2:
            text = "mean: n/a"
        else:
            mean = float(np.mean(data))
            std = float(np.std(data, ddof=1))
            std_raw = float(np.std(rawd, ddof=1))
            pct = (lambda v: f"  ({100.0 * v / abs(mean):.2f} %)") if abs(mean) > 1e-9 else (lambda v: "")
            text = (f"mean {1e3 * mean:.3f} mV\n"
                    f"std  {1e3 * std:.3f} mV{pct(std)} filtered\n"
                    f"std  {1e3 * std_raw:.3f} mV{pct(std_raw)} raw ADC\n"
                    f"n = {data.size}" + (f"  ({n_excl} held excl.)" if n_excl else ""))
        if text != self._window_stats_state:
            self._window_stats_state = text
            self.lbl_window_stats.setText(text)

    def _update_deviation_axis(self):
        """Relabel the right axis of the input plot as percent of setpoint."""
        y0, y1 = self.plot_input.getViewBox().viewRange()[1]
        sp = float(self.sp_data.view()[-1]) if len(self.sp_data) else self.sp_setpoint.value()
        state = (y0, y1, sp)
        if state == self._dev_axis_state:
            return
        self._dev_axis_state = state
        if abs(sp) < 1e-9:
            self._dev_axis.setStyle(showValues=False)
            self._dev_axis.setLabel("Deviation from setpoint (setpoint is 0)")
            return
        self._dev_axis.setStyle(showValues=True)
        self._dev_axis.setLabel("Deviation from setpoint", units="%")
        scale = 100.0 / abs(sp)
        self._dev_axis.setRange((y0 - sp) * scale, (y1 - sp) * scale)

    def _visible_slice(self):
        """Return (time, input) restricted to the current rolling window."""
        x = self.t_data.view()
        if x.size == 0:
            return x, x
        i0 = int(np.searchsorted(x, x[-1] - self._window_s, side="left"))
        return x[i0:], self.input_data.view()[i0:]

    def refresh_plots(self):
        x_all = self.t_data.view()
        if x_all.size == 0:
            return
        t_end = float(x_all[-1])
        t_start = t_end - self._window_s
        # Keep one sample left of the window edge so the trace enters the view
        # from the side instead of popping in.
        i0 = max(0, int(np.searchsorted(x_all, t_start, side="left")) - 1)
        x = x_all[i0:]
        self.curve_input.setData(x, self.input_data.view()[i0:])
        self.curve_sp.setData(x, self.sp_data.view()[i0:])
        self.curve_target_out.setData(x, self.target_out_data.view()[i0:])
        self.curve_actual_out.setData(x, self.actual_out_data.view()[i0:])
        if self._raw_visible:
            self.curve_raw.setData(x, self.raw_data.view()[i0:])
        self._refresh_hold_regions(x, self.hold_data.view()[i0:])
        self._refresh_window_stats(self.input_data.view()[i0:], self.hold_data.view()[i0:],
                                   self.raw_data.view()[i0:])
        self._refresh_ext_live()

        # Pin the time axis to the rolling window (plot_output is x-linked).
        self._setting_range = True
        try:
            self.plot_input.setXRange(t_start, t_end, padding=0)
            self._view_pinned = True
        finally:
            self._setting_range = False

        self._update_deviation_axis()
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

        # Record bar: folder, note, start/stop button, status
        self._settings = QSettings("redPitaLock", "monitor_rp")
        rec_bar = QHBoxLayout()
        self.btn_rec_folder = QPushButton("Folder…")
        self.btn_rec_folder.setToolTip("Choose the folder the HDF5 recordings are written to")
        self.btn_rec_folder.clicked.connect(self._choose_record_folder)
        rec_bar.addWidget(self.btn_rec_folder)
        self.lbl_rec_folder = QLabel()
        self.lbl_rec_folder.setMinimumWidth(120)
        rec_bar.addWidget(self.lbl_rec_folder)
        rec_bar.addSpacing(12)
        rec_bar.addWidget(QLabel("Note:"))
        self.txt_rec_note = QLineEdit()
        self.txt_rec_note.setPlaceholderText("optional, stored in the file")
        self.txt_rec_note.setMinimumWidth(220)
        rec_bar.addWidget(self.txt_rec_note, 1)
        self.btn_record = QPushButton("● Start recording")
        self.btn_record.setToolTip(
            "Record the telemetry of both channels, all board parameters and every\n"
            "command sent into a new .h5 file in the chosen folder. Press again to stop and save.")
        self.btn_record.setMinimumWidth(150)
        self.btn_record.clicked.connect(self._toggle_recording)
        rec_bar.addWidget(self.btn_record)
        self.lbl_rec_status = QLabel("")
        self.lbl_rec_status.setMinimumWidth(170)
        rec_bar.addWidget(self.lbl_rec_status)
        layout.addLayout(rec_bar)
        self._update_record_folder_label()

        # Tabs for channels
        self.tabs = QTabWidget()
        self.channel_panels: list[ChannelPanel] = []
        for i in range(2):
            panel = ChannelPanel(i)
            panel.window_changed.connect(self._on_panel_window_changed)
            self.channel_panels.append(panel)
            self.tabs.addTab(panel, f"Channel {i + 1}")
        layout.addWidget(self.tabs)

        # Menu bar
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("&File")
        self.act_record_folder = QAction("Recording &folder…", self)
        self.act_record_folder.triggered.connect(self._choose_record_folder)
        file_menu.addAction(self.act_record_folder)
        self.act_record_start = QAction("Start &recording (HDF5)", self)
        self.act_record_start.setToolTip(
            "Record telemetry of both channels, board parameters and commands to a .h5 file."
        )
        self.act_record_start.triggered.connect(self._start_recording)
        file_menu.addAction(self.act_record_start)
        self.act_record_stop = QAction("S&top recording and save", self)
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
            "Fast PSD (V²/Hz) from 125 MS/s ADC bulk-read, computed on the Red Pitaya ARM."
        )
        self.act_psd.toggled.connect(self._on_psd_toggled)
        view_menu.addAction(self.act_psd)
        self.act_raw = QAction("Show &raw ADC samples", self)
        self.act_raw.setCheckable(True)
        self.act_raw.setChecked(True)
        self.act_raw.setToolTip("Overlay the unfiltered ADC sample of each telemetry tick on the input plot")
        self.act_raw.toggled.connect(self._on_raw_toggled)
        view_menu.addAction(self.act_raw)
        view_menu.addSeparator()

        history_widget = QWidget()
        history_layout = QHBoxLayout(history_widget)
        history_layout.setContentsMargins(8, 2, 8, 2)
        history_layout.addWidget(QLabel("History window (s):"))
        self.sp_history_sec = QDoubleSpinBox()
        self.sp_history_sec.setRange(MIN_HISTORY_SECONDS, MAX_HISTORY_SECONDS)
        self.sp_history_sec.setDecimals(1)
        self.sp_history_sec.setSingleStep(1.0)
        self.sp_history_sec.setValue(DEFAULT_HISTORY_SECONDS)
        self.sp_history_sec.setToolTip(
            "Length of the rolling time window shown in the plots, in seconds. "
            "Zooming the plots with the mouse wheel changes it as well."
        )
        self.sp_history_sec.editingFinished.connect(self._on_history_changed)
        history_layout.addWidget(self.sp_history_sec)
        history_action = QWidgetAction(self)
        history_action.setDefaultWidget(history_widget)
        view_menu.addAction(history_action)
        self._history_seconds = DEFAULT_HISTORY_SECONDS

        tools_menu = menu_bar.addMenu("&Tools")
        self.act_optimize = QAction("&Optimize PID gains (target < 1 %)…", self)
        self.act_optimize.setToolTip(
            "Runs scripts/optimize_pid.py in a console window for the current channel tab. "
            "The PID must be on. Gains are changed on the board while it runs.")
        self.act_optimize.triggered.connect(self._launch_optimizer)
        tools_menu.addAction(self.act_optimize)
        self.act_step_test = QAction("&Step response test (measure only)…", self)
        self.act_step_test.triggered.connect(lambda: self._launch_tool("optimize_step.py", ["--dry-run"]))
        tools_menu.addAction(self.act_step_test)
        self.act_optimize_step = QAction("Optimize step response (target < &1 ms)…", self)
        self.act_optimize_step.triggered.connect(lambda: self._launch_tool("optimize_step.py", ["--target-ms", "1.0"]))
        tools_menu.addAction(self.act_optimize_step)
        self.act_sweep = QAction("S&weep PID gains (fluctuation heatmap)…", self)
        self.act_sweep.setToolTip("Measures the fluctuation on a kp x ki grid around the current gains and shows it live as a heatmap")
        self.act_sweep.triggered.connect(lambda: self._launch_tool("sweep_pid.py", []))
        tools_menu.addAction(self.act_sweep)
        tools_menu.addSeparator()
        self.act_refresh_params = QAction("&Refresh parameters from board", self)
        self.act_refresh_params.triggered.connect(self._refresh_params_from_board)
        tools_menu.addAction(self.act_refresh_params)

        self._recorder: Optional[H5Recorder] = None
        self._latest_params: list[dict] = [{}, {}]
        self._rec_timer = QTimer()
        self._rec_timer.setInterval(1000)
        self._rec_timer.timeout.connect(self._record_tick)

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
        self.worker.data_batch.connect(self._on_data_batch)
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

    def _refresh_params_from_board(self):
        """Re-read all channel parameters (e.g. after the optimizer changed gains)."""
        if self.worker is None:
            self.status.showMessage("Not connected")
            return
        for ch in range(len(self.channel_panels)):
            self.worker.send_command(f"GET {ch} params")
        self.status.showMessage("Parameters refreshed from board")

    def _launch_optimizer(self):
        self._launch_tool("optimize_pid.py", ["--target", "1.0"])

    def _launch_tool(self, script_name, extra_args):
        """Start one of the scripts for the current channel in its own console."""
        import subprocess
        ch = self.tabs.currentIndex()
        panel = self.channel_panels[ch]
        if not panel.btn_enable.isChecked():
            self.status.showMessage("Switch the PID on first (this tool needs a closed loop)")
            return
        host = self.txt_host.text().strip()
        port = self.txt_port.text().strip() or str(DEFAULT_PORT)
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), script_name)
        cmd = [sys.executable, script, host, port, "--ch", str(ch)] + list(extra_args)
        try:
            if sys.platform.startswith("win"):
                # "cmd /k" keeps the console open after the script ends, so
                # its output (and any error) stays readable.
                subprocess.Popen(["cmd", "/k"] + cmd,
                                 creationflags=subprocess.CREATE_NEW_CONSOLE,
                                 cwd=os.path.dirname(script))
            else:
                subprocess.Popen(cmd, cwd=os.path.dirname(script))
            self.status.showMessage(
                f"{script_name} started for channel {ch + 1} in a separate console; "
                "use Tools > Refresh parameters when it is done")
        except OSError as e:
            self.status.showMessage(f"Could not start {script_name}: {e}")

    @Slot()
    def _on_connected(self):
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
            if self._recorder is not None:
                self._recorder.add_command(cmd)

    @Slot(int, dict)
    def _on_params(self, ch, kv):
        if 0 <= ch < len(self.channel_panels):
            self._latest_params[ch] = dict(kv)
            self.channel_panels[ch].sync_from_params(kv)

    @Slot(int, float, float, float, float, float, int, int, int)
    def _on_data(self, ch, time_s, input_v, target_out_v, actual_out_v, setpoint_v,
                 enabled, holding=0, gate=0, level=-1, raw_v=None):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].add_sample(
                time_s, input_v, target_out_v, actual_out_v, setpoint_v, enabled,
                holding, gate, level, raw_v
            )
            if len(self.channel_panels) == 2:
                # This channel's input is the other channel's control voltage.
                self.channel_panels[1 - ch].set_control_voltage(input_v)

    @Slot(list)
    def _on_data_batch(self, batch):
        """All telemetry samples of one socket read (the plots only get the
        latest per channel; the recording keeps every one)."""
        rec = self._recorder
        if rec is None:
            return
        for ch, ts, iv, tgt, act, sp, en, hold, gate, level, raw in batch:
            rec.add_sample(ch, ts, iv, raw, tgt, act, sp, en, hold, gate, level)

    @Slot(int, int, float, object)
    def _on_psd(self, ch, n_bins, fs, bins):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].set_psd_data(n_bins, fs, bins)

    @Slot(bool)
    def _on_stats_toggled(self, checked: bool):
        for panel in self.channel_panels:
            panel.set_statistics_visible(checked)

    @Slot(bool)
    def _on_raw_toggled(self, checked: bool):
        for panel in self.channel_panels:
            panel.set_raw_visible(checked)

    @Slot(bool)
    def _on_psd_toggled(self, checked: bool):
        for panel in self.channel_panels:
            panel.set_psd_visible(checked)

    def _on_history_changed(self):
        self._apply_window_seconds(self.sp_history_sec.value())

    @Slot(float)
    def _on_panel_window_changed(self, secs: float):
        """A plot was zoomed: mirror the new window in the spinbox and all panels."""
        self._apply_window_seconds(secs)

    def _apply_window_seconds(self, secs: float):
        secs = float(np.clip(secs, MIN_HISTORY_SECONDS, MAX_HISTORY_SECONDS))
        self._history_seconds = secs
        for panel in self.channel_panels:
            panel.set_window_seconds(secs)
        if abs(self.sp_history_sec.value() - secs) > 0.05:
            self.sp_history_sec.blockSignals(True)
            self.sp_history_sec.setValue(secs)
            self.sp_history_sec.blockSignals(False)
        self.status.showMessage(f"Rolling window: {secs:.1f} s")

    # ......................................................... recording

    def _record_folder(self) -> str:
        return str(self._settings.value("record_folder", "") or "")

    def _update_record_folder_label(self):
        folder = self._record_folder()
        if folder:
            shown = folder if len(folder) <= 45 else "…" + folder[-44:]
            self.lbl_rec_folder.setText(shown)
            self.lbl_rec_folder.setToolTip(folder)
        else:
            self.lbl_rec_folder.setText("(no folder chosen)")
            self.lbl_rec_folder.setToolTip("")

    def _choose_record_folder(self) -> bool:
        folder = QFileDialog.getExistingDirectory(
            self, "Folder for HDF5 recordings", self._record_folder() or os.path.expanduser("~"))
        if not folder:
            return False
        self._settings.setValue("record_folder", folder)
        self._update_record_folder_label()
        return True

    def _fetch_params_blocking(self, timeout_ms: int = 800):
        """Ask the board for the parameters of both channels and wait until they
        arrived (or the timeout passed) so the recording metadata is current."""
        if self.worker is None or not self.worker.isRunning():
            return
        n_ch = len(self.channel_panels)
        loop = QEventLoop()
        got = set()

        def _on_p(ch, kv):
            got.add(ch)
            if len(got) >= n_ch:
                loop.quit()

        self.worker.params_received.connect(_on_p)
        try:
            for ch in range(n_ch):
                self.worker.send_command(f"GET {ch} params")
            QTimer.singleShot(timeout_ms, loop.quit)
            loop.exec()
        finally:
            self.worker.params_received.disconnect(_on_p)

    def _toggle_recording(self):
        if self._recorder is None:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._recorder is not None:
            self.status.showMessage("Already recording — stop first.")
            return
        folder = self._record_folder()
        if not folder or not os.path.isdir(folder):
            if not self._choose_record_folder():
                return
            folder = self._record_folder()
        self._fetch_params_blocking()
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(folder, f"redpitalock_{stamp}.h5")
        k = 1
        while os.path.exists(path):              # never overwrite an earlier run
            path = os.path.join(folder, f"redpitalock_{stamp}_{k}.h5")
            k += 1
        host = self.txt_host.text().strip()
        port = self.txt_port.text().strip() or str(DEFAULT_PORT)
        try:
            rec = H5Recorder(path, host, int(port), self.txt_rec_note.text().strip())
        except ImportError:
            self.status.showMessage("Recording needs the h5py package:  pip install h5py")
            return
        except OSError as e:
            self.status.showMessage(f"Could not create {path}: {e}")
            return
        rec.set_attr("history_window_s", self._history_seconds)
        rec.set_attr("connected", 1 if (self.worker is not None and self.worker.isRunning()) else 0)
        for ch, kv in enumerate(self._latest_params):
            rec.set_params(ch, kv, "start")
        self._recorder = rec
        self._rec_timer.start()
        self.btn_record.setText("■ Stop and save")
        self.btn_record.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
        self.act_record_stop.setEnabled(True)
        self.act_record_start.setEnabled(False)
        self.btn_rec_folder.setEnabled(False)
        self.act_record_folder.setEnabled(False)
        self.lbl_rec_status.setText("● REC 00:00")
        self.status.showMessage(f"Recording to {path}")

    def _record_tick(self):
        rec = self._recorder
        if rec is None:
            return
        try:
            rec.flush()
        except (OSError, ValueError) as e:
            self.status.showMessage(f"Recording error: {e}")
            self._stop_recording()
            return
        secs = int(rec.elapsed_s())
        n = sum(rec.n_samples)
        self.lbl_rec_status.setText(f"● REC {secs // 60:02d}:{secs % 60:02d}   {n} samples")

    def _stop_recording(self):
        rec = self._recorder
        if rec is None:
            return
        self._rec_timer.stop()
        self._recorder = None          # stop feeding samples before the final flush
        try:
            self._fetch_params_blocking()
            rec.set_attr("note", self.txt_rec_note.text().strip())
            for ch, kv in enumerate(self._latest_params):
                rec.set_params(ch, kv, "end")
        finally:
            try:
                rec.close()
                msg = (f"Saved {rec.path}  ({rec.elapsed_s():.1f} s, "
                       f"{rec.n_samples[0]} + {rec.n_samples[1]} samples)")
            except (OSError, ValueError) as e:
                msg = f"Error while saving {rec.path}: {e}"
        self.btn_record.setText("● Start recording")
        self.btn_record.setStyleSheet("")
        self.act_record_stop.setEnabled(False)
        self.act_record_start.setEnabled(True)
        self.btn_rec_folder.setEnabled(True)
        self.act_record_folder.setEnabled(True)
        self.lbl_rec_status.setText("")
        self.status.showMessage(msg)

    @Slot(int, float, float, float, float, float)
    def _on_autotune_done(self, ch, Ku, Tu, Kp, Ki, Kd):
        if 0 <= ch < len(self.channel_panels):
            self.channel_panels[ch].on_autotune_done(Ku, Tu, Kp, Ki, Kd)
            self.status.showMessage(
                f"Ch{ch+1} autotune complete: Ku={Ku:.4f} Tu={Tu:.4f}s  →  Kp={Kp:.4f} Ki={Ki:.4f} Kd={Kd:.6f}")

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

    if len(sys.argv) >= 2:
        QTimer.singleShot(200, window._on_connect)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()

"""
sweep_pid.py -- sweep kp and ki around the current gains, measure the relative
intensity fluctuation (std / setpoint, in percent) for every combination and
show the result live as a heatmap.

The grid is logarithmic: n points per axis spanning +/- `range` decades around
the current gains (default 7 x 7 over 0.1x .. 10x). Each cell is measured at
the loop rate with the firmware's capture facility (see optimize_pid.py), held
samples excluded. Cells where the output saturates or a spectral line appears
(oscillation) are hatched with a marker; their number is still shown.

Window controls:
  * Apply best      send the gains of the best cell to the board
  * Restore original  put the starting gains back
  * click a cell    apply that cell's gains
  * Close           the original gains are restored unless you applied a cell

Usage:
    python sweep_pid.py rp-f0f9f8.local [--ch 0] [--n 7] [--range 1.0] [--measure 2]
    python sweep_pid.py rp-f0f9f8.local --kp-range 0.5 --ki-range 1.5 --n 9
"""

import argparse
import math
import os
import sys
import time

import numpy as np

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")
from PySide6.QtCore import Qt, QThread, Signal, Slot          # noqa: E402
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QMainWindow,  # noqa: E402
                               QPushButton, QVBoxLayout, QWidget)
import pyqtgraph as pg                                         # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optimize_pid as op                                      # noqa: E402  (Client, measure_capture)


class SweepWorker(QThread):
    cell_done = Signal(int, int, float, str)      # i (kp index), j (ki index), rel %, flag text
    status = Signal(str)
    finished_all = Signal()

    def __init__(self, host, port, ch, kp_vals, ki_vals, settle, measure, capture_ms, orig):
        super().__init__()
        self.host, self.port, self.ch = host, port, ch
        self.kp_vals, self.ki_vals = kp_vals, ki_vals
        self.settle, self.measure, self.capture_ms = settle, measure, capture_ms
        self.orig = orig
        self._stop = False
        self.client = None
        self.sp = None
        self.out_min, self.out_max = 0.0, 1.0

    def stop(self):
        self._stop = True

    def apply(self, kp, ki):
        c = self.client
        c.send(f"SET {self.ch} kp {kp:.6g}")
        c.send(f"SET {self.ch} ki {ki:.6g}")
        need = 1.2 / ki if ki > 0 else 0.0
        if self.orig["integral_max"] < need:
            c.send(f"SET {self.ch} integral_max {need:.6g}")

    def run(self):
        try:
            self.client = op.Client(self.host, self.port)
            kv = self.client.params(self.ch)
            self.sp = float(kv.get("setpoint", 0.0))
            self.out_min = float(kv.get("out_min", 0.0))
            self.out_max = float(kv.get("out_max", 1.0))
            if int(kv.get("enabled", "0")) != 1:
                self.status.emit("PID is off: switch it on first")
                return
            total = len(self.kp_vals) * len(self.ki_vals)
            k = 0
            # walk the grid outward from the centre so the mild cells come first
            order = sorted(((i, j) for i in range(len(self.kp_vals)) for j in range(len(self.ki_vals))),
                           key=lambda ij: abs(ij[0] - len(self.kp_vals) // 2) + abs(ij[1] - len(self.ki_vals) // 2))
            for i, j in order:
                if self._stop:
                    break
                kp, ki = self.kp_vals[i], self.ki_vals[j]
                self.apply(kp, ki)
                k += 1
                self.status.emit(f"[{k}/{total}] kp={kp:.4g} ki={ki:.4g} ...")
                m = op.measure_capture(self.client, self.ch, self.settle, self.measure,
                                       self.out_min, self.out_max, self.sp, self.capture_ms)
                if not m["valid"]:
                    self.cell_done.emit(i, j, float("nan"), m["reason"])
                    continue
                flag = ""
                if m["sat"] > 0.02:
                    flag = "sat"
                elif m["osc_ratio"] > 15:
                    flag = f"osc {m['osc_f']:.0f} Hz"
                self.cell_done.emit(i, j, 100.0 * m["rel"], flag)
        except Exception as e:                                   # noqa: BLE001
            self.status.emit(f"error: {e}")
        finally:
            try:
                if self.client is not None:
                    self.apply(self.orig["kp"], self.orig["ki"])
                    self.client.send(f"SET {self.ch} integral_max {self.orig['integral_max']:.6g}")
            except OSError:
                pass
            self.finished_all.emit()


class SweepWindow(QMainWindow):
    def __init__(self, args, orig, kp_vals, ki_vals):
        super().__init__()
        self.args, self.orig = args, orig
        self.kp_vals, self.ki_vals = kp_vals, ki_vals
        self.grid = np.full((len(kp_vals), len(ki_vals)), np.nan)
        self.flags = {}
        self.applied = None
        self.setWindowTitle(f"PID sweep ch{args.ch + 1}: fluctuation (%) vs kp, ki  "
                            f"[start kp={orig['kp']:.4g} ki={orig['ki']:.4g}]")
        self.resize(900, 720)

        central = QWidget(); self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "ki (log10)")
        self.plot.setLabel("left", "kp (log10)")
        self.plot.showGrid(x=False, y=False)
        # The colour bar needs an ImageItem; the cells themselves are drawn
        # as rectangles in data coordinates, so colour and number can never
        # disagree on where a cell is.
        self.img = pg.ImageItem(axisOrder="row-major")
        self.img.setVisible(False)
        self.plot.addItem(self.img)
        self.cells = {}
        # image geometry: cell (i, j) covers one log-decade step
        self.lkp = np.log10(kp_vals); self.lki = np.log10(ki_vals)
        dkp = (self.lkp[1] - self.lkp[0]) if len(kp_vals) > 1 else 1.0
        dki = (self.lki[1] - self.lki[0]) if len(ki_vals) > 1 else 1.0
        self.img.setRect(pg.QtCore.QRectF(self.lki[0] - dki / 2, self.lkp[0] - dkp / 2,
                                          dki * len(ki_vals), dkp * len(kp_vals)))
        self.cmap = pg.colormap.get("viridis")
        self.bar = pg.ColorBarItem(label="fluctuation (%)", colorMap=self.cmap, interactive=False)
        self.bar.setImageItem(self.img, insert_in=self.plot.getPlotItem())
        self.plot.setAspectLocked(False)
        self.plot.setXRange(self.lki[0] - dki / 2, self.lki[-1] + dki / 2, padding=0.02)
        self.plot.setYRange(self.lkp[0] - dkp / 2, self.lkp[-1] + dkp / 2, padding=0.02)
        # ticks with the real values
        self.plot.getAxis("bottom").setTicks([[(v, f"{10 ** v:.3g}") for v in self.lki]])
        self.plot.getAxis("left").setTicks([[(v, f"{10 ** v:.3g}") for v in self.lkp]])
        self.texts = {}
        self.best_marker = pg.ScatterPlotItem(size=18, pen=pg.mkPen("w", width=2), brush=None, symbol="o")
        self.plot.addItem(self.best_marker)
        self.start_marker = pg.ScatterPlotItem(size=14, pen=pg.mkPen("#ff5722", width=2), brush=None, symbol="s")
        self.start_marker.setData([math.log10(orig["ki"])], [math.log10(orig["kp"])])
        self.plot.addItem(self.start_marker)
        self.plot.scene().sigMouseClicked.connect(self._on_click)
        layout.addWidget(self.plot)

        row = QHBoxLayout()
        self.lbl = QLabel("starting ...")
        self.btn_best = QPushButton("Apply best"); self.btn_best.setEnabled(False)
        self.btn_best.clicked.connect(self._apply_best)
        self.btn_restore = QPushButton("Restore original")
        self.btn_restore.clicked.connect(self._restore)
        self.btn_stop = QPushButton("Stop sweep")
        self.btn_stop.clicked.connect(self._stop)
        row.addWidget(self.lbl, 1); row.addWidget(self.btn_stop); row.addWidget(self.btn_best); row.addWidget(self.btn_restore)
        layout.addLayout(row)
        hint = QLabel("white circle = best cell, orange square = starting gains, "
                      "click a cell to apply its gains; 'sat' = output saturated, 'osc' = oscillation line")
        hint.setStyleSheet("color: #888")
        layout.addWidget(hint)

        self.worker = SweepWorker(args.host, args.port, args.ch, kp_vals, ki_vals,
                                  args.settle, args.measure, args.capture_ms, orig)
        self.worker.cell_done.connect(self._on_cell)
        self.worker.status.connect(self.lbl.setText)
        self.worker.finished_all.connect(self._on_finished)
        self.worker.start()
        self._running = True

    # ---------------------------------------------------------------- slots
    @Slot(int, int, float, str)
    def _on_cell(self, i, j, rel, flag):
        self.grid[i, j] = rel
        self.flags[(i, j)] = flag
        self._redraw()

    def _redraw(self):
        vals = self.grid[np.isfinite(self.grid)]
        if vals.size:
            lo, hi = float(vals.min()), float(min(vals.max(), vals.min() * 10 + 1e-9))
            if hi <= lo:
                hi = lo + 1e-6
            img = np.where(np.isfinite(self.grid), self.grid, lo)
            self.img.setImage(img, levels=(lo, hi), autoLevels=False)
            self.bar.setLevels((lo, hi))
            dkp = (self.lkp[1] - self.lkp[0]) if len(self.lkp) > 1 else 1.0
            dki = (self.lki[1] - self.lki[0]) if len(self.lki) > 1 else 1.0
            for (i, j), f in self.flags.items():
                v = self.grid[i, j]
                if (i, j) not in self.cells:
                    r = pg.QtWidgets.QGraphicsRectItem(self.lki[j] - dki / 2, self.lkp[i] - dkp / 2, dki, dkp)
                    r.setPen(pg.mkPen(None))
                    r.setZValue(-5)
                    self.plot.addItem(r)
                    self.cells[(i, j)] = r
                if np.isfinite(v):
                    frac = float(np.clip((v - lo) / (hi - lo), 0.0, 1.0))
                    self.cells[(i, j)].setBrush(pg.mkBrush(self.cmap.mapToQColor(frac)))
                else:
                    self.cells[(i, j)].setBrush(pg.mkBrush(60, 60, 60))
            # best cell: lowest fluctuation among unflagged cells
            best = None
            for (i, j), f in self.flags.items():
                if not f and np.isfinite(self.grid[i, j]):
                    if best is None or self.grid[i, j] < self.grid[best]:
                        best = (i, j)
            if best is not None:
                self.best = best
                self.best_marker.setData([self.lki[best[1]]], [self.lkp[best[0]]])
                self.btn_best.setEnabled(True)
        # numbers in the cells
        for (i, j), f in self.flags.items():
            if (i, j) not in self.texts:
                t = pg.TextItem(anchor=(0.5, 0.5), color="w")
                t.setPos(self.lki[j], self.lkp[i])
                self.plot.addItem(t)
                self.texts[(i, j)] = t
            v = self.grid[i, j]
            s = f"{v:.2f}" if np.isfinite(v) else "n/a"
            if f:
                s += f"\n{f}"
            self.texts[(i, j)].setText(s)

    def _on_click(self, ev):
        if ev.button() != Qt.LeftButton:
            return
        pos = self.plot.getPlotItem().vb.mapSceneToView(ev.scenePos())
        j = int(np.argmin(np.abs(self.lki - pos.x())))
        i = int(np.argmin(np.abs(self.lkp - pos.y())))
        if abs(self.lki[j] - pos.x()) > 0.6 or abs(self.lkp[i] - pos.y()) > 0.6:
            return
        self._apply(self.kp_vals[i], self.ki_vals[j])

    def _apply(self, kp, ki):
        if self._running:
            self.lbl.setText("stop the sweep first, then apply")
            return
        try:
            c = op.Client(self.args.host, self.args.port)
            c.send(f"SET {self.args.ch} kp {kp:.6g}")
            c.send(f"SET {self.args.ch} ki {ki:.6g}")
            need = 1.2 / ki if ki > 0 else 0.0
            if self.orig["integral_max"] < need:
                c.send(f"SET {self.args.ch} integral_max {need:.6g}")
            c.sock.close()
            self.applied = (kp, ki)
            self.lbl.setText(f"applied kp={kp:.4g} ki={ki:.4g}  (GUI: Tools > Refresh parameters)")
        except OSError as e:
            self.lbl.setText(f"could not apply: {e}")

    def _apply_best(self):
        if getattr(self, "best", None) is not None:
            i, j = self.best
            self._apply(self.kp_vals[i], self.ki_vals[j])

    def _restore(self):
        self._apply(self.orig["kp"], self.orig["ki"])
        self.applied = None
        self.lbl.setText(f"restored kp={self.orig['kp']:.4g} ki={self.orig['ki']:.4g}")

    def _stop(self):
        self.worker.stop()
        self.lbl.setText("stopping after the current cell ...")

    def _on_finished(self):
        self._running = False
        self.btn_stop.setEnabled(False)
        done = int(np.isfinite(self.grid).sum())
        msg = f"sweep finished: {done} cells measured; original gains restored"
        if getattr(self, "best", None) is not None:
            i, j = self.best
            msg += f" | best kp={self.kp_vals[i]:.4g} ki={self.ki_vals[j]:.4g}: {self.grid[i, j]:.2f} %"
        self.lbl.setText(msg)

    def closeEvent(self, ev):
        if self._running:
            self.worker.stop()
            self.worker.wait(15000)
        ev.accept()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host")
    ap.add_argument("port", nargs="?", type=int, default=5000)
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--n", type=int, default=7, help="grid points per axis")
    ap.add_argument("--range", type=float, default=1.0, help="decades around the current gains (both axes)")
    ap.add_argument("--kp-range", type=float, default=None, help="decades for kp (overrides --range)")
    ap.add_argument("--ki-range", type=float, default=None, help="decades for ki (overrides --range)")
    ap.add_argument("--settle", type=float, default=1.0, help="settle time after a gain change (s)")
    ap.add_argument("--measure", type=float, default=2.0, help="measurement time per cell (s)")
    ap.add_argument("--capture-ms", type=float, default=100.0)
    args = ap.parse_args()

    c = op.Client(args.host, args.port)
    kv = c.params(args.ch)
    c.sock.close()
    orig = {"kp": float(kv.get("kp", 1.0)), "ki": float(kv.get("ki", 1.0)),
            "integral_max": float(kv.get("integral_max", 0.5))}
    if int(kv.get("enabled", "0")) != 1:
        print("PID is off. Switch it on in the GUI first.")
        return 2
    if orig["kp"] <= 0 or orig["ki"] <= 0:
        print("kp and ki must be > 0 for a logarithmic sweep (set a starting point first).")
        return 2
    rkp = args.kp_range if args.kp_range is not None else args.range
    rki = args.ki_range if args.ki_range is not None else args.range
    kp_vals = orig["kp"] * np.logspace(-rkp, rkp, args.n)
    ki_vals = orig["ki"] * np.logspace(-rki, rki, args.n)
    print(f"sweep ch{args.ch + 1}: kp {kp_vals[0]:.4g} .. {kp_vals[-1]:.4g}, ki {ki_vals[0]:.4g} .. {ki_vals[-1]:.4g}, "
          f"{args.n}x{args.n} cells, ~{args.n * args.n * (args.settle + args.measure):.0f} s")
    app = QApplication.instance() or QApplication([])
    win = SweepWindow(args, orig, kp_vals, ki_vals)
    win.show()
    app.exec()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as e:
        print(f"ERROR: {e}")
        sys.exit(2)

"""
optimize_step.py -- measure and optimise the step response of the redPitaLock
loop (how fast the photodiode voltage follows a setpoint change).

The firmware records a setpoint step at the full loop rate (SET <ch> step), so
the timing resolution is the loop period, not the 100 Hz telemetry. Each test
is a step up by `delta` followed by a step back down; the reported numbers are
the worse of the two.

Metrics (per step):
  rise     10 % -> 90 % of the step
  settle   time after the step until the input stays within +/-band of the new
           setpoint for the rest of the capture (band = 1 % of the setpoint)
  overshoot  peak excursion beyond the new setpoint, in percent of the step

Optimisation: coordinate descent on log(kp) and log(ki) (and kd with --kd),
minimising the settle time. Candidates that overshoot more than --max-overshoot,
do not settle inside the capture, or saturate the output are rejected and the
previous best is restored. Stops when the settle time is below --target-ms,
when the step is exhausted or after --max-evals evaluations. Ctrl-C restores
the original gains.

Usage:
    python optimize_step.py rp-f0f9f8.local --dry-run           # one test, print + save CSV
    python optimize_step.py rp-f0f9f8.local                     # optimise to < 1 ms
    python optimize_step.py rp-f0f9f8.local --delta-pct 5 --capture-ms 10 --target-ms 0.5
"""

import argparse
import csv
import math
import os
import socket
import sys
import time

import numpy as np


class Client:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(0.2)
        self.buf = ""

    def send(self, cmd):
        self.sock.sendall((cmd + "\n").encode())

    def lines(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            try:
                data = self.sock.recv(1 << 20)
            except socket.timeout:
                continue
            if not data:
                raise RuntimeError("server closed connection")
            self.buf += data.decode("utf-8", errors="replace")
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                yield line.strip()

    def params(self, ch):
        self.send(f"GET {ch} params")
        for line in self.lines(2.0):
            if line.startswith(f"P {ch} "):
                return dict(tok.split("=", 1) for tok in line.split()[2:] if "=" in tok)
        raise RuntimeError("no P line received (daemon not running?)")

    def step(self, ch, delta_v, capture_ms, pre_ms, timeout=8.0):
        """Trigger a setpoint step and return (t_us, input, output, pre_n, period_us, sp_before, status)."""
        self.send(f"SET {ch} step_ms {int(capture_ms)}")
        self.send(f"SET {ch} step_pre_ms {int(pre_ms)}")
        self.send(f"SET {ch} step {delta_v:.6f}")
        hdr = None
        rows = []
        t_end = time.time() + timeout
        for line in self.lines(timeout):
            if hdr is None:
                if line.startswith("ERR"):
                    raise RuntimeError(f"step refused: {line}")
                if line.startswith(f"STEP {ch} "):
                    p = line.split()
                    hdr = {"n": int(p[2]), "pre_n": int(p[3]), "period_us": int(p[4]),
                           "sp_before": float(p[5]), "delta": float(p[6]), "status": p[7]}
                    if hdr["n"] == 0:
                        break
                continue
            if line.startswith("S "):
                p = line.split()
                rows.append((float(p[1]), float(p[2]), float(p[3])))
                if len(rows) >= hdr["n"]:
                    break
            if time.time() > t_end:
                break
        if hdr is None:
            raise RuntimeError("no STEP capture received (firmware without step support?)")
        a = np.array(rows, dtype=float) if rows else np.zeros((0, 3))
        return a[:, 0], a[:, 1], a[:, 2], hdr


def step_metrics(t_us, x, hdr, band_frac):
    """Rise, settle and overshoot for one captured step.

    The photodiode trace is noisy, so:
      * rise / settle / overshoot are evaluated on a ~200 us moving average,
      * the settling band is widened to 4 sigma of the averaged noise when the
        requested band is narrower (noise measured on the pre-trigger samples),
      * "settled" tolerates 2 % stray samples outside the band afterwards,
      * ringing is detected spectrally (a dominant line in the residual after
        the step), which white noise cannot fake.
    """
    pre_n = hdr["pre_n"]
    sp_before, delta = hdr["sp_before"], hdr["delta"]
    final = sp_before + delta
    res = {"ok": False, "reason": ""}
    if hdr["status"] != "ok":
        res["reason"] = f"capture {hdr['status']} (hold or PID off during the step)"
        return res
    if x.size < pre_n + 8 or abs(delta) < 1e-9:
        res["reason"] = "capture too short"
        return res
    period = float(np.median(np.diff(t_us))) if t_us.size > 2 else float(hdr["period_us"])
    w = max(3, int(round(200.0 / max(period, 1.0))))          # ~200 us smoothing
    w = min(w, max(3, (x.size - pre_n) // 10))

    def smooth(sig, win):
        pad = np.pad(sig, (win // 2, win - 1 - win // 2), mode="edge")
        return np.convolve(pad, np.ones(win) / win, mode="valid")

    xs = smooth(x, w)
    base = float(np.mean(x[:pre_n])) if pre_n > 0 else sp_before
    noise = float(np.std(x[:pre_n], ddof=1)) if pre_n >= 5 else 0.0
    noise_s = noise / math.sqrt(w)                             # averaged-noise estimate
    if pre_n >= 4 * w:                                         # use the measured one if better
        noise_s = max(noise_s, float(np.std(xs[:pre_n], ddof=1)))

    post_t, post_x = t_us[pre_n:], xs[pre_n:]
    valid = max(1, post_x.size - w)                            # drop the truncated tail
    post_t, post_x = post_t[:valid], post_x[:valid]
    sgn = 1.0 if delta > 0 else -1.0
    y = sgn * (post_x - base) / abs(delta)                     # 0 = before, 1 = new setpoint

    def first_cross(level):
        idx = np.flatnonzero(y >= level)
        return float(post_t[idx[0]]) if idx.size else float("nan")
    t10, t90 = first_cross(0.1), first_cross(0.9)
    rise = t90 - t10 if not (math.isnan(t10) or math.isnan(t90)) else float("nan")

    band_req = band_frac * abs(final) if abs(final) > 1e-9 else band_frac * abs(delta)
    band = max(band_req, 4.0 * noise_s)
    inside = np.abs(post_x - final) <= band
    # earliest point that starts a run of w samples inside the band and after
    # which at least 98 % of the samples stay inside
    n_post = inside.size
    suffix_inside = np.cumsum(inside[::-1])[::-1]              # count inside from i to end
    settle = float("inf")
    run = np.convolve(inside.astype(int), np.ones(w, dtype=int), mode="valid") == w
    for i in np.flatnonzero(run):
        if suffix_inside[i] >= 0.98 * (n_post - i):
            settle = float(post_t[i])
            break
    overshoot = 100.0 * max(0.0, float(np.max(y)) - 1.0)
    overshoot_noise = 100.0 * 4.0 * noise_s / abs(delta)       # what noise alone can produce

    # ringing: dominant spectral line in the residual after the step. The
    # residual is the raw trace minus a heavy moving average (which follows
    # the step itself but not an oscillation); the first 4w samples after the
    # edge are skipped because the average smears the edge there.
    xs4 = smooth(x, 4 * w)
    resid = (x - xs4)[pre_n + 4 * w:]
    half = (x[pre_n:] - final)[(x.size - pre_n) // 2:]
    osc_f, osc_ratio = 0.0, 0.0
    if resid.size >= 32:
        spec = np.abs(np.fft.rfft((resid - resid.mean()) * np.hanning(resid.size)))
        freqs = np.fft.rfftfreq(resid.size, d=period * 1e-6)
        sel = freqs > 0
        s_sel, f_sel = spec[sel], freqs[sel]
        med = float(np.median(s_sel)) or 1e-12
        k = int(np.argmax(s_sel))
        osc_ratio, osc_f = float(s_sel[k] / med), float(f_sel[k])

    res.update({"ok": True, "rise_us": rise, "settle_us": settle, "overshoot_pct": overshoot,
                "overshoot_noise_pct": overshoot_noise,
                "band_v": band, "band_noise_limited": band > band_req * 1.001, "noise_v": noise,
                "osc_f": osc_f, "osc_ratio": osc_ratio,
                "final_err_pct": 100.0 * (float(np.mean(half)) / abs(final) if abs(final) > 1e-9 else 0.0),
                "period_us": period, "n": int(x.size), "smooth_w": w})
    return res


def fmt(m):
    if not m["ok"]:
        return f"invalid ({m['reason']})"
    s = "never" if math.isinf(m["settle_us"]) else f"{m['settle_us'] / 1000:.3f} ms"
    r = "n/a" if math.isnan(m["rise_us"]) else f"{m['rise_us'] / 1000:.3f} ms"
    nb = f" (band widened to +/-{1e3 * m['band_v']:.2f} mV by noise)" if m.get("band_noise_limited") else ""
    return (f"settle {s}, rise {r}, overshoot {m['overshoot_pct']:.0f} %, "
            f"residual error {m['final_err_pct']:+.2f} %, "
            f"ringing peak {m['osc_ratio']:.0f}x @ {m['osc_f']:.0f} Hz, "
            f"loop period {m['period_us']:.0f} us, n={m['n']}{nb}")


def run_test(c, ch, delta_v, capture_ms, pre_ms, band_frac, pause=0.3):
    """Step up then down. Returns (worst metrics, [(hdr, t, x, out, metrics), ...])."""
    caps = []
    for d in (delta_v, -delta_v):
        t, x, out, hdr = c.step(ch, d, capture_ms, pre_ms)
        caps.append((hdr, t, x, out, step_metrics(t, x, hdr, band_frac)))
        time.sleep(pause)
    ms = [cp[4] for cp in caps]
    if not all(m["ok"] for m in ms):
        bad = next(m for m in ms if not m["ok"])
        return bad, caps
    worst = max(ms, key=lambda m: (m["settle_us"], m["overshoot_pct"]))
    worst = dict(worst)
    worst["overshoot_pct"] = max(m["overshoot_pct"] for m in ms)
    worst["overshoot_noise_pct"] = max(m["overshoot_noise_pct"] for m in ms)
    worst["osc_ratio"] = max(m["osc_ratio"] for m in ms)
    return worst, caps


def show_plot(caps, title, band_frac):
    """Open a window with the captured step(s): input, setpoint and output vs time."""
    try:
        # PySide6 must be imported before pyqtgraph: when pyqtgraph picks the
        # Qt binding itself on this machine it lands on a broken one.
        os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")
        from PySide6.QtWidgets import QApplication
        import pyqtgraph as pg
    except ImportError as e:
        print(f"  (no plot: {e})")
        return
    app = QApplication.instance() or QApplication([])
    pg.setConfigOptions(antialias=True)
    win = pg.GraphicsLayoutWidget(title=title)
    win.resize(1000, 700)
    colors = ("#2196F3", "#E91E63")
    p_in = win.addPlot(title="Photodiode input (step up / step down)")
    p_in.setLabel("left", "Voltage", units="V"); p_in.setLabel("bottom", "Time after step", units="s")
    p_in.addLegend(offset=(10, 10)); p_in.showGrid(x=True, y=True, alpha=0.3)
    win.nextRow()
    p_out = win.addPlot(title="Output voltage (DAC)")
    p_out.setLabel("left", "Voltage", units="V"); p_out.setLabel("bottom", "Time after step", units="s")
    p_out.addLegend(offset=(10, 10)); p_out.showGrid(x=True, y=True, alpha=0.3)
    p_out.setXLink(p_in)
    for k, (hdr, t, x, out, m) in enumerate(caps):
        ts = t * 1e-6
        name = "step up" if hdr["delta"] > 0 else "step down"
        p_in.plot(ts, x, pen=pg.mkPen(colors[k % 2], width=2), name=name)
        sp = np.where(np.arange(t.size) >= hdr["pre_n"], hdr["sp_before"] + hdr["delta"], hdr["sp_before"])
        p_in.plot(ts, sp, pen=pg.mkPen(colors[k % 2], width=1, style=pg.QtCore.Qt.DashLine))
        final = hdr["sp_before"] + hdr["delta"]
        band = band_frac * abs(final)
        p_in.addItem(pg.InfiniteLine(final + band, angle=0, pen=pg.mkPen("#888", style=pg.QtCore.Qt.DotLine)))
        p_in.addItem(pg.InfiniteLine(final - band, angle=0, pen=pg.mkPen("#888", style=pg.QtCore.Qt.DotLine)))
        p_out.plot(ts, out, pen=pg.mkPen(colors[k % 2], width=2), name=name)
        if m.get("ok") and not math.isinf(m["settle_us"]):
            p_in.addItem(pg.InfiniteLine(m["settle_us"] * 1e-6, angle=90,
                                         pen=pg.mkPen(colors[k % 2], style=pg.QtCore.Qt.DotLine),
                                         label=f"settled {m['settle_us'] / 1000:.2f} ms",
                                         labelOpts={"position": 0.9 - 0.1 * k, "color": colors[k % 2]}))
    p_in.addItem(pg.InfiniteLine(0, angle=90, pen=pg.mkPen("#aaa")))
    win.show()
    print("  close the plot window to finish.")
    app.exec()


def save_csv(path, caps):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "t_us", "input_V", "output_V", "setpoint_V"])
        for k, (hdr, t, x, out, _) in enumerate(caps):
            for i in range(t.size):
                sp = hdr["sp_before"] + (hdr["delta"] if i >= hdr["pre_n"] else 0.0)
                w.writerow([k, f"{t[i]:.1f}", f"{x[i]:.6f}", f"{out[i]:.6f}", f"{sp:.6f}"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host")
    ap.add_argument("port", nargs="?", type=int, default=5000)
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--delta-pct", type=float, default=5.0, help="step size in percent of the setpoint")
    ap.add_argument("--capture-ms", type=float, default=20.0, help="capture length after the step")
    ap.add_argument("--pre-ms", type=float, default=1.0, help="pre-trigger length")
    ap.add_argument("--band", type=float, default=1.0, help="settling band in percent of the setpoint")
    ap.add_argument("--target-ms", type=float, default=1.0, help="target settle time")
    ap.add_argument("--max-overshoot", type=float, default=30.0, help="reject candidates above this (%)")
    ap.add_argument("--max-evals", type=int, default=40)
    ap.add_argument("--osc-ratio", type=float, default=6.0,
                    help="spectral peak / median in the residual above which a candidate counts as ringing")
    ap.add_argument("--step", type=float, default=2.0, help="initial multiplicative step of the descent")
    ap.add_argument("--no-grid", action="store_true",
                    help="skip the coarse kp x ki grid scan that precedes the descent")
    ap.add_argument("--min-step", type=float, default=1.1)
    ap.add_argument("--kd", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="one test only, change nothing")
    ap.add_argument("--save", default=None, help="CSV path for the (best) capture")
    ap.add_argument("--no-plot", action="store_true", help="do not open the plot window at the end")
    args = ap.parse_args()
    ch = args.ch
    band = args.band / 100.0
    target_us = args.target_ms * 1000.0

    c = Client(args.host, args.port)
    kv = c.params(ch)
    f = lambda k, d=0.0: float(kv.get(k, d))
    orig = {"kp": f("kp"), "ki": f("ki"), "kd": f("kd"), "integral_max": f("integral_max", 0.5)}
    sp = f("setpoint")
    period_us = int(f("loop_rate", 1000))
    actual_us = f("loop_actual", 0.0)
    if actual_us > 0:
        period_us = int(round(actual_us))     # what the CPU really achieves
    delta_v = abs(sp) * args.delta_pct / 100.0
    print(f"Connected to {args.host}:{args.port}, channel {ch + 1}")
    print(f"  gains kp={orig['kp']:.4g} ki={orig['ki']:.4g} kd={orig['kd']:.4g}; setpoint {1e3 * sp:.2f} mV; "
          f"loop period set {kv.get('loop_rate', '?')} us / measured {period_us} us; "
          f"step +/-{1e3 * delta_v:.2f} mV ({args.delta_pct} %)")
    if int(kv.get("enabled", "0")) != 1:
        print("PID is off. Switch it on in the GUI first.")
        return 2
    if abs(sp) < 1e-6:
        print("Setpoint is 0: cannot define a relative step.")
        return 2
    if period_us > target_us / 5:
        print(f"  NOTE: a {args.target_ms} ms settle time needs a loop period of about "
              f"{target_us / 10:.0f} us or less; the loop runs at {period_us} us. "
              f"Set 'Loop period' in the GUI accordingly.")
    if int(kv.get("hold_mode", "0")) != 0:
        print("  NOTE: a hold during a step aborts that capture; the test is repeated if that happens.")

    def apply(g):
        c.send(f"SET {ch} kp {g['kp']:.6g}")
        c.send(f"SET {ch} ki {g['ki']:.6g}")
        c.send(f"SET {ch} kd {g['kd']:.6g}")
        # Only make sure the integrator clamp is not the limiting element
        # (it must allow at least the full output swing); never shrink it,
        # tiny values round to 0 in the GUI and switch the integrator off.
        need = 1.2 / g["ki"] if g["ki"] > 0 else 0.0
        if orig["integral_max"] < need:
            c.send(f"SET {ch} integral_max {need:.6g}")

    def test():
        for attempt in range(3):
            m, caps = run_test(c, ch, delta_v, args.capture_ms, args.pre_ms, band)
            if m["ok"] or "aborted" not in m.get("reason", ""):
                return m, caps
            time.sleep(1.0)
        return m, caps

    print("\nBaseline step test ...")
    base, base_caps = test()
    print("  " + fmt(base))
    if base["ok"] and base["noise_v"] > 0 and delta_v < 10 * base["noise_v"]:
        print(f"  NOTE: the step ({1e3 * delta_v:.2f} mV) is less than 10x the loop noise "
              f"({1e3 * base['noise_v']:.2f} mV rms); settle times will be coarse. "
              f"Consider --delta-pct {min(50, math.ceil(10 * base['noise_v'] / abs(sp) * 100)):d} "
              f"or a quieter loop first.")
    save_path = args.save or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                          f"step_capture_ch{ch + 1}.csv")
    if args.dry_run or not base["ok"]:
        if base["ok"]:
            save_csv(save_path, base_caps)
            print(f"  capture saved to {save_path}")
            print("Dry run: nothing changed.")
            if not args.no_plot:
                show_plot(base_caps, f"Step response ch{ch + 1}: {fmt(base)}", band)
            return 0 if base["settle_us"] < target_us else 1
        print("Cannot optimise: " + base["reason"])
        return 2

    def score(m):
        return m["settle_us"] if m["ok"] else float("inf")

    def acceptable(m):
        if not m["ok"]:
            return False, m["reason"]
        if math.isinf(m["settle_us"]):
            return False, "does not settle inside the capture"
        if m["overshoot_pct"] > max(args.max_overshoot, m.get("overshoot_noise_pct", 0.0)):
            return False, f"overshoot {m['overshoot_pct']:.0f} %"
        if m["osc_ratio"] > args.osc_ratio:
            return False, f"ringing at {m['osc_f']:.0f} Hz ({m['osc_ratio']:.0f}x)"
        return True, ""

    best, best_m, best_caps = dict(orig), base, base_caps
    evals, step = 0, args.step
    try:
        if not args.no_grid:
            # Coarse global scan: a factor 0.3 .. 10 around the starting gains in
            # both directions. The descent that follows only finds the nearest
            # local optimum, so this is what gets it out of a bad start.
            print("\nGrid scan (kp x 0.3..10, ki x 0.1..10) ...")
            for fkp in (0.3, 1.0, 3.0, 10.0):
                for fki in (0.1, 0.3, 1.0, 3.0, 10.0):
                    if fkp == 1.0 and fki == 1.0:
                        continue
                    cand = dict(best)
                    cand["kp"] = orig["kp"] * fkp
                    cand["ki"] = orig["ki"] * fki
                    if not (1e-3 <= cand["kp"] <= 1e4 and 0 <= cand["ki"] <= 1e6):
                        continue
                    apply(cand)
                    time.sleep(0.5)
                    evals += 1
                    m, caps = test()
                    good, why = acceptable(m)
                    tag = f"[grid] kp={cand['kp']:.4g} ki={cand['ki']:.4g}: "
                    if good and score(m) < score(best_m) * 0.95:
                        print(tag + fmt(m) + "  <- best so far")
                        best, best_m, best_caps = cand, m, caps
                    else:
                        print(tag + (fmt(m) if good else f"rejected: {why}"))
            apply(best)
            time.sleep(0.5)
            print(f"  grid best: kp={best['kp']:.4g} ki={best['ki']:.4g} -> {fmt(best_m)}")
        order = ["kp", "ki"] + (["kd"] if args.kd else [])
        while step >= args.min_step and evals < args.max_evals and score(best_m) >= target_us:
            improved = False
            for name in order:
                if score(best_m) < target_us:
                    break
                for factor in (step, 1.0 / step):
                    cand = dict(best)
                    if name == "kd" and cand["kd"] <= 0:
                        cand["kd"] = 1e-4 if factor > 1 else 0.0
                    else:
                        cand[name] *= factor
                    if not (1e-3 <= cand["kp"] <= 1e4 and 0 <= cand["ki"] <= 1e6 and 0 <= cand["kd"] <= 10):
                        continue
                    apply(cand)
                    time.sleep(0.5)
                    evals += 1
                    m, caps = test()
                    good, why = acceptable(m)
                    tag = (f"[{evals:2d}] {name}{'x' if factor > 1 else '/'}{step:.2f} -> "
                           f"kp={cand['kp']:.4g} ki={cand['ki']:.4g} kd={cand['kd']:.4g}: ")
                    if good and score(m) < score(best_m) * 0.95:
                        print(tag + fmt(m) + "  <- better")
                        best, best_m, best_caps = cand, m, caps
                        improved = True
                        break
                    print(tag + (fmt(m) if good else f"rejected: {why}"))
                    apply(best)
                    time.sleep(0.5)
                if evals >= args.max_evals:
                    break
            if not improved:
                step = math.sqrt(step)
                print(f"  no improvement, step -> {step:.3f}")
        if score(best_m) < target_us:
            print("\nTarget reached, confirming ...")
            m, caps = test()
            print("  " + fmt(m))
            if acceptable(m)[0]:
                best_m, best_caps = m, caps
    except KeyboardInterrupt:
        print("\nInterrupted: restoring the original gains.")
        apply(orig)
        return 130

    apply(best)
    save_csv(save_path, best_caps)
    print("\n==== result ====")
    print(f"  best gains : kp={best['kp']:.4g} ki={best['ki']:.4g} kd={best['kd']:.4g}")
    print(f"  step response: {fmt(best_m)}")
    print(f"  started at   : {fmt(base)}")
    print(f"  capture saved to {save_path}")
    reached = best_m["ok"] and best_m["settle_us"] < target_us
    if not args.no_plot and best_m["ok"]:
        show_plot(best_caps, f"Step response ch{ch + 1} after optimisation: {fmt(best_m)}", band)
    if reached:
        print(f"  target of {args.target_ms} ms reached.")
    else:
        print(f"  target of {args.target_ms} ms NOT reached. Likely limits: loop period ({period_us} us), "
              "the input filter in the firmware, the AOM/RF/photodiode response, or the output limits.")
    print("  The gains are left on the board. In the GUI use Tools > Refresh parameters from board.")
    return 0 if reached else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as e:
        print(f"ERROR: {e}")
        sys.exit(2)

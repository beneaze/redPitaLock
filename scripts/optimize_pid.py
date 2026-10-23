"""
optimize_pid.py -- automatic PID gain optimisation for redPitaLock.

Goal: bring the relative intensity fluctuation (standard deviation of the
photodiode voltage divided by the setpoint) below a target, 1 % by default.

The script talks to the stabilizer daemon over TCP, exactly like the GUI, so
it can run while the GUI is open. The fluctuation is measured at the full loop
rate using the firmware's capture facility (a setpoint "step" of 0 V records
the input at loop rate), so oscillations far above the 100 Hz telemetry rate
are seen; --telemetry falls back to the 100 Hz stream. It only touches kp, ki, kd and integral_max
of the chosen channel; setpoint, output limits and the sample-and-hold setup
stay as they are. Samples taken while the loop is holding (beam gated off)
are excluded from every measurement.

Method:
  1. measure the fluctuation with the current gains (baseline)
  2. coordinate descent on log(ki) and log(kp): try x1.5 and /1.5, keep the
     better one, shrink the step when neither direction helps
  3. a candidate is rejected (and the previous best restored) when the output
     saturates, when a spectral line appears (onset of oscillation) or when
     the fluctuation explodes
  4. stop when the target is reached and confirmed by a longer measurement,
     when the step is exhausted, or after --max-evals evaluations

Whenever ki changes, integral_max is set to 1.2 / ki so the anti-windup clamp
keeps protecting the loop (the firmware rescales the integrator on a ki change,
so the output does not jump).

Usage:
    python optimize_pid.py rp-f0f9f8.local                 # optimise channel 1
    python optimize_pid.py rp-f0f9f8.local --dry-run       # only measure
    python optimize_pid.py rp-f0f9f8.local --ch 1 --target 0.5 --measure 6
    python optimize_pid.py rp-f0f9f8.local --enable        # switch the PID on first

Ctrl-C restores the gains that were active when the script started.
"""

import argparse
import math
import socket
import sys
import time

import numpy as np

ADC_LSB_V = 2.0 / 16384.0          # 14 bit over +/-1 V (LV input)
TELEMETRY_HZ = 100.0               # nominal D-line rate


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
                data = self.sock.recv(65536)
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
                kv = {}
                for tok in line.split()[2:]:
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        kv[k] = v
                return kv
        raise RuntimeError("no P line received (daemon not running?)")

    def samples(self, ch, seconds):
        """(t, input, actual_out, setpoint, enabled, holding) rows for `seconds`."""
        rows = []
        for line in self.lines(seconds):
            if not line.startswith(f"D {ch} "):
                continue
            p = line.split()
            if len(p) < 10:
                raise RuntimeError("firmware without sample-and-hold telemetry; update it")
            rows.append((float(p[2]), float(p[3]), float(p[5]), float(p[6]),
                         int(p[7]), int(p[8])))
        return np.array(rows, dtype=float) if rows else np.zeros((0, 6))

    def capture(self, ch, capture_ms, timeout=8.0):
        """Loop-rate recording of (t_us, input, output) via a 0 V step; None if aborted."""
        self.send(f"SET {ch} step_ms {int(capture_ms)}")
        self.send(f"SET {ch} step_pre_ms 0")
        self.send(f"SET {ch} step 0")
        hdr, rows = None, []
        for line in self.lines(timeout):
            if hdr is None:
                if line.startswith("ERR"):
                    raise RuntimeError(f"capture refused: {line}")
                if line.startswith(f"STEP {ch} "):
                    p = line.split()
                    hdr = {"n": int(p[2]), "status": p[7], "period_us": int(p[4])}
                    if hdr["n"] == 0:
                        break
                continue
            if line.startswith("S "):
                p = line.split()
                rows.append((float(p[1]), float(p[2]), float(p[3])))
                if len(rows) >= hdr["n"]:
                    break
        if hdr is None:
            raise RuntimeError("no capture received (firmware without step support?)")
        if hdr["status"] != "ok":
            return None
        return np.array(rows, dtype=float)


def measure_capture(c, ch, settle, duration, out_min, out_max, setpoint, capture_ms=100.0):
    """Fluctuation measured at the loop rate from repeated 0 V 'step' captures."""
    c.samples(ch, settle)                      # let the loop settle, discard
    chunks, t_end, aborted = [], time.time() + duration, 0
    while time.time() < t_end:
        a = c.capture(ch, capture_ms)
        if a is None:
            aborted += 1                       # a hold happened: skip this chunk
            time.sleep(0.2)
            continue
        chunks.append(a)
    if not chunks:
        return {"n": 0, "valid": False, "reason": f"no capture outside holds ({aborted} aborted)"}
    x = np.concatenate([a[:, 1] for a in chunks])
    out = np.concatenate([a[:, 2] for a in chunks])
    dts = np.concatenate([np.diff(a[:, 0]) for a in chunks if a.shape[0] > 2])
    fs = 1e6 / float(np.median(dts)) if dts.size else TELEMETRY_HZ
    n = x.size
    sp_v = setpoint
    mean = float(np.mean(x))
    std = float(np.std(x, ddof=1))
    ref = abs(sp_v) if abs(sp_v) > 1e-6 else abs(mean)
    rel = std / ref if ref > 0 else float("inf")
    offset = (mean - sp_v) / ref if ref > 0 else float("inf")
    sat = float(np.mean((out <= out_min + 5e-4) | (out >= out_max - 5e-4)))
    # spectrum of the longest chunk (contiguous samples)
    a = max(chunks, key=lambda q: q.shape[0])
    xd = (a[:, 1] - np.mean(a[:, 1])) * np.hanning(a.shape[0])
    spec = np.abs(np.fft.rfft(xd))
    freqs = np.fft.rfftfreq(a.shape[0], d=1.0 / fs)
    band = freqs > 20.0
    osc_f, osc_ratio = 0.0, 0.0
    if band.sum() > 8:
        s_band = spec[band]
        med = float(np.median(s_band)) or 1e-12
        k = int(np.argmax(s_band))
        osc_ratio = float(s_band[k] / med)
        osc_f = float(freqs[band][k])
    return {"n": n, "valid": True, "reason": "", "rel": rel, "std": std, "mean": mean,
            "setpoint": sp_v, "offset": offset, "sat": sat, "osc_f": osc_f,
            "osc_ratio": osc_ratio, "fs": fs}


def measure(c, ch, settle, duration, out_min, out_max):
    """Return a dict describing the fluctuation with the current gains (100 Hz telemetry)."""
    c.samples(ch, settle)                      # let the loop settle, discard
    d = c.samples(ch, duration)
    res = {"n": 0, "valid": False, "reason": "no telemetry"}
    if d.shape[0] < 20:
        return res
    t, x, out, sp, en, hold = d.T
    # drop held samples and 0.3 s after each hold (resume transient)
    keep = hold == 0
    hold_end = np.flatnonzero(np.diff(hold.astype(int)) == -1)
    for i in hold_end:
        keep &= ~((t > t[i]) & (t <= t[i] + 0.3))
    if en.min() < 1:
        return {"n": int(keep.sum()), "valid": False, "reason": "PID is off"}
    x, out, sp, t = x[keep], out[keep], sp[keep], t[keep]
    n = x.size
    if n < 0.5 * duration * TELEMETRY_HZ * 0.5:
        return {"n": n, "valid": False, "reason": "too few samples outside holds"}
    sp_v = float(np.median(sp))
    mean = float(np.mean(x))
    std = float(np.std(x, ddof=1))
    ref = abs(sp_v) if abs(sp_v) > 1e-6 else abs(mean)
    rel = std / ref if ref > 0 else float("inf")
    offset = (mean - sp_v) / ref if ref > 0 else float("inf")
    sat = float(np.mean((out <= out_min + 5e-4) | (out >= out_max - 5e-4)))
    # oscillation: a dominant spectral line in the (detrended) input
    fs = (n - 1) / (t[-1] - t[0]) if t[-1] > t[0] else TELEMETRY_HZ
    xd = (x - mean) * np.hanning(n)
    spec = np.abs(np.fft.rfft(xd))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    band = freqs > 0.5
    osc_f, osc_ratio = 0.0, 0.0
    if band.sum() > 8:
        s_band = spec[band]
        med = float(np.median(s_band)) or 1e-12
        k = int(np.argmax(s_band))
        osc_ratio = float(s_band[k] / med)
        osc_f = float(freqs[band][k])
    return {"n": n, "valid": True, "reason": "", "rel": rel, "std": std, "mean": mean,
            "setpoint": sp_v, "offset": offset, "sat": sat, "osc_f": osc_f,
            "osc_ratio": osc_ratio, "fs": fs}


def fmt(m):
    if not m["valid"]:
        return f"invalid ({m['reason']}, n={m['n']})"
    return (f"{100 * m['rel']:.3f} %  (std {1e3 * m['std']:.3f} mV, mean {1e3 * m['mean']:.2f} mV, "
            f"offset {100 * m['offset']:+.2f} %, sat {100 * m['sat']:.1f} %, "
            f"peak {m['osc_ratio']:.0f}x @ {m['osc_f']:.0f} Hz, fs {m['fs']:.0f} Hz, n={m['n']})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host")
    ap.add_argument("port", nargs="?", type=int, default=5000)
    ap.add_argument("--ch", type=int, default=0, help="channel index (0 = IN1/OUT1)")
    ap.add_argument("--target", type=float, default=1.0, help="target fluctuation in percent")
    ap.add_argument("--settle", type=float, default=2.0, help="settle time after a gain change (s)")
    ap.add_argument("--measure", type=float, default=4.0, help="measurement window (s)")
    ap.add_argument("--max-evals", type=int, default=40)
    ap.add_argument("--step", type=float, default=2.0, help="initial multiplicative step of the descent")
    ap.add_argument("--no-grid", action="store_true",
                    help="skip the coarse kp x ki grid scan that precedes the descent")
    ap.add_argument("--min-step", type=float, default=1.1, help="stop when the step shrinks below this")
    ap.add_argument("--kd", action="store_true", help="also tune kd after kp/ki converged")
    ap.add_argument("--dry-run", action="store_true", help="measure only, change nothing")
    ap.add_argument("--enable", action="store_true", help="switch the PID on before starting")
    ap.add_argument("--osc-ratio", type=float, default=15.0,
                    help="spectral peak / median above which a candidate counts as oscillating")
    ap.add_argument("--max-sat", type=float, default=0.02, help="allowed saturated fraction")
    ap.add_argument("--telemetry", action="store_true",
                    help="measure from the 100 Hz telemetry instead of loop-rate captures")
    args = ap.parse_args()
    ch = args.ch
    target = args.target / 100.0

    c = Client(args.host, args.port)
    kv = c.params(ch)
    f = lambda k, d=0.0: float(kv.get(k, d))
    out_min, out_max = f("out_min", 0.0), f("out_max", 1.0)
    orig = {"kp": f("kp"), "ki": f("ki"), "kd": f("kd"), "integral_max": f("integral_max", 0.5)}
    sp = f("setpoint")
    print(f"Connected to {args.host}:{args.port}, channel {ch + 1}")
    print(f"  current gains: kp={orig['kp']:.4g} ki={orig['ki']:.4g} kd={orig['kd']:.4g} "
          f"integral_max={orig['integral_max']:.4g}; setpoint {1e3 * sp:.2f} mV; "
          f"loop period set {kv.get('loop_rate', '?')} us / measured {kv.get('loop_actual', '?')} us; "
          f"hold_mode {kv.get('hold_mode', '?')}")
    if abs(sp) > 1e-9:
        print(f"  ADC quantisation alone: {100 * ADC_LSB_V / abs(sp):.3f} % of the setpoint "
              f"(raise the signal level if this is not far below the target)")
    if int(kv.get("enabled", "0")) != 1:
        if args.enable and not args.dry_run:
            print("  enabling PID ...")
            c.send(f"SET {ch} enabled 1")
            time.sleep(1.0)
        else:
            print("PID is off. Switch it on in the GUI or pass --enable.")
            return 2

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

    def meas(duration):
        if args.telemetry:
            return measure(c, ch, args.settle, duration, out_min, out_max)
        return measure_capture(c, ch, args.settle, duration, out_min, out_max, sp)

    print("\nBaseline measurement ...")
    base = meas(args.measure)
    print("  " + fmt(base))
    if args.dry_run or not base["valid"]:
        if not base["valid"]:
            print("Cannot optimise: " + base["reason"])
            return 2
        print("Dry run: nothing changed.")
        return 0 if base["rel"] < target else 1

    best = dict(orig)
    best_m = base
    evals = 0
    step = args.step
    history = [("start", dict(orig), base)]

    def ok(m, ref_rel):
        if not m["valid"]:
            return False, m["reason"]
        if m["sat"] > args.max_sat:
            return False, f"output saturates {100 * m['sat']:.0f} % of the time"
        if m["osc_ratio"] > args.osc_ratio:
            return False, f"oscillation at {m['osc_f']:.1f} Hz ({m['osc_ratio']:.0f}x)"
        if m["rel"] > 5 * ref_rel and m["rel"] > target:
            return False, "fluctuation exploded"
        return True, ""

    try:
        if not args.no_grid:
            # Coarse global scan first; the descent only finds the nearest
            # local optimum.
            print("\nGrid scan (kp x 0.3..3, ki x 0.1..10) ...")
            for fkp in (0.3, 1.0, 3.0):
                for fki in (0.1, 0.3, 1.0, 3.0, 10.0):
                    if fkp == 1.0 and fki == 1.0:
                        continue
                    cand = dict(best)
                    cand["kp"] = orig["kp"] * fkp
                    cand["ki"] = orig["ki"] * fki
                    if not (1e-3 <= cand["kp"] <= 1e3 and 0 <= cand["ki"] <= 1e5):
                        continue
                    apply(cand)
                    evals += 1
                    m = meas(args.measure)
                    good, why = ok(m, best_m["rel"])
                    tag = f"[grid] kp={cand['kp']:.4g} ki={cand['ki']:.4g}: "
                    history.append((tag, cand, m))
                    if good and m["rel"] < best_m["rel"] * 0.98:
                        print(tag + fmt(m) + "  <- best so far")
                        best, best_m = cand, m
                    else:
                        print(tag + (fmt(m) if good else f"rejected: {why}"))
            apply(best)
            time.sleep(0.5)
            print(f"  grid best: kp={best['kp']:.4g} ki={best['ki']:.4g} -> {fmt(best_m)}")
        params_order = ["ki", "kp"] + (["kd"] if args.kd else [])
        while step >= args.min_step and evals < args.max_evals:
            improved = False
            for name in params_order:
                if best_m["rel"] < target:
                    break
                for factor in (step, 1.0 / step):
                    cand = dict(best)
                    if name == "kd" and cand["kd"] <= 0:
                        cand["kd"] = 1e-4 if factor > 1 else 0.0
                    else:
                        cand[name] = cand[name] * factor
                    if not (1e-3 <= cand["kp"] <= 1e3 and 0 <= cand["ki"] <= 1e5 and 0 <= cand["kd"] <= 10):
                        continue
                    apply(cand)
                    evals += 1
                    m = meas(args.measure)
                    good, why = ok(m, best_m["rel"])
                    tag = f"[{evals:2d}] {name}{'x' if factor > 1 else '/'}{step:.2f} -> kp={cand['kp']:.4g} ki={cand['ki']:.4g} kd={cand['kd']:.4g}: "
                    history.append((tag, cand, m))
                    if good and m["rel"] < best_m["rel"] * 0.98:
                        print(tag + fmt(m) + "  <- better")
                        best, best_m = cand, m
                        improved = True
                        break
                    print(tag + (fmt(m) if good else f"rejected: {why}"))
                    apply(best)          # restore the best before the next try
                    time.sleep(0.5)
                if evals >= args.max_evals:
                    break
            if best_m["rel"] < target:
                print(f"\nTarget reached ({100 * best_m['rel']:.3f} % < {args.target} %), confirming with a longer measurement ...")
                conf = meas(2 * args.measure)
                print("  " + fmt(conf))
                if conf["valid"] and conf["rel"] < target and ok(conf, best_m["rel"])[0]:
                    best_m = conf
                    break
                best_m = conf if conf["valid"] else best_m
            if not improved:
                step = math.sqrt(step)
                print(f"  no improvement, step -> {step:.3f}")
    except KeyboardInterrupt:
        print("\nInterrupted: restoring the original gains.")
        apply(orig)
        return 130

    apply(best)
    print("\n==== result ====")
    print(f"  best gains : kp={best['kp']:.4g} ki={best['ki']:.4g} kd={best['kd']:.4g}")
    print(f"  fluctuation: {100 * best_m['rel']:.3f} %  (started at {100 * base['rel']:.3f} %)")
    reached = best_m["rel"] < target
    if reached:
        print(f"  target of {args.target} % reached.")
    else:
        print(f"  target of {args.target} % NOT reached. Likely limits:")
        if abs(sp) > 1e-9 and ADC_LSB_V / abs(sp) > 0.3 * target:
            print("   - ADC quantisation / sensor noise: raise the photodiode signal level")
        print(f"   - loop bandwidth: loop period set {kv.get('loop_rate', '?')} us, "
              f"measured {kv.get('loop_actual', '?')} us; a setting the CPU cannot meet just adds jitter")
        print("   - noise above the loop bandwidth (check the PSD view)")
    print("  The gains are left on the board. In the GUI use Tools > Refresh parameters from board.")
    return 0 if reached else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as e:
        print(f"ERROR: {e}")
        sys.exit(2)

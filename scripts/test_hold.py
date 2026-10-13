"""
test_hold.py -- exercise the sample-and-hold logic of the redPitaLock daemon
over TCP, without the GUI.

Safe part (always runs):
  * checks that the firmware reports the hold parameters in GET params
  * asserts a manual hold and checks the telemetry gate flag follows it
    (the gate is reported even while the PID is off)

Loop part (only with --enable, because it turns the PID on and drives OUT<ch+1>):
  * enables the PID, asserts a manual hold, verifies "holding" goes to 1 and
    that the actual DAC output stays constant for the hold duration
  * releases the hold, verifies "holding" returns to 0 and the output moves
    again, then disables the PID

Usage:
    python test_hold.py rp-f0f492.local            # safe checks only
    python test_hold.py rp-f0f492.local --enable   # also run the loop test
    python test_hold.py rp-f0f492.local --ch 1 --hold-seconds 2
"""

import argparse
import socket
import sys
import time


class Client:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(0.2)
        self.buf = ""

    def send(self, cmd):
        self.sock.sendall((cmd + "\n").encode())

    def lines(self, seconds):
        """Yield complete lines received within the next `seconds`."""
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
        raise RuntimeError("no P line received")

    def samples(self, ch, seconds):
        """Collect (t, input, target, actual, enabled, holding, gate) tuples."""
        out = []
        for line in self.lines(seconds):
            if not line.startswith(f"D {ch} "):
                continue
            p = line.split()
            if len(p) < 10:
                raise RuntimeError(
                    f"D line has {len(p)} fields; firmware without sample-and-hold? {line!r}")
            out.append((float(p[2]), float(p[3]), float(p[4]), float(p[5]),
                        int(p[7]), int(p[8]), int(p[9])))
        return out


def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    return bool(cond)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host")
    ap.add_argument("port", nargs="?", type=int, default=5000)
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--enable", action="store_true",
                    help="also run the loop test (turns the PID on!)")
    ap.add_argument("--hold-seconds", type=float, default=1.0)
    args = ap.parse_args()
    ch = args.ch
    ok = True

    c = Client(args.host, args.port)
    print(f"Connected to {args.host}:{args.port}, channel {ch}")

    # --- 1. parameters -----------------------------------------------------
    kv = c.params(ch)
    needed = ["hold_mode", "hold", "hold_pin", "hold_pol", "hold_thresh",
              "hold_hyst", "hold_debounce", "resume_delay", "hold_park",
              "hold_park_v", "hold_count"]
    missing = [k for k in needed if k not in kv]
    ok &= check(not missing, f"GET params reports hold keys (missing: {missing})")
    print("        " + " ".join(f"{k}={kv.get(k)}" for k in needed))

    # Make sure we start from a clean state.
    c.send(f"SET {ch} hold 0")
    c.send(f"SET {ch} hold_mode 0")
    c.send(f"SET {ch} hold_count_reset 0")

    # --- 2. manual gate while PID off ------------------------------------
    s = c.samples(ch, 0.5)
    ok &= check(s and all(x[6] == 0 for x in s), "gate idle before manual hold")
    c.send(f"SET {ch} hold 1")
    time.sleep(0.1)
    s = c.samples(ch, 0.5)
    ok &= check(s and all(x[6] == 1 for x in s[len(s) // 2:]), "gate asserted after SET hold 1")
    ok &= check(s and all(x[5] == 0 for x in s), "not 'holding' while PID is off")
    c.send(f"SET {ch} hold 0")
    time.sleep(0.1)
    s = c.samples(ch, 0.5)
    ok &= check(s and all(x[6] == 0 for x in s[len(s) // 2:]), "gate released after SET hold 0")

    if not args.enable:
        print("\nLoop test skipped (pass --enable to turn the PID on and test the hold).")
        return 0 if ok else 1

    # --- 3. hold with the loop running -----------------------------------
    print("\nEnabling PID for the loop test ...")
    c.send(f"SET {ch} enabled 1")
    time.sleep(0.5)
    c.samples(ch, 0.3)  # flush

    c.send(f"SET {ch} hold 1")
    time.sleep(0.1)
    s = c.samples(ch, args.hold_seconds)
    held = [x for x in s if x[5] == 1]
    ok &= check(len(held) > 0.8 * len(s), f"holding=1 while held ({len(held)}/{len(s)} samples)")
    if held:
        outs = [x[3] for x in held]
        spread = max(outs) - min(outs)
        ok &= check(spread < 1e-6, f"actual output constant while holding (spread {spread:.2e} V)")

    c.send(f"SET {ch} hold 0")
    time.sleep(0.2)
    s = c.samples(ch, 1.0)
    ok &= check(s and all(x[5] == 0 for x in s[len(s) // 2:]), "holding=0 after release")

    kv = c.params(ch)
    ok &= check(int(kv.get("hold_count", "0")) >= 1, f"hold_count incremented ({kv.get('hold_count')})")

    c.send(f"SET {ch} enabled 0")
    print("PID disabled again.")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as e:
        print(f"ERROR: {e}")
        sys.exit(2)

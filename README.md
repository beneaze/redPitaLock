# redPitaLock

Two-channel PID intensity stabilizer for the **Red Pitaya STEMlab 125-14**,
with a Qt-based remote control GUI.

The system reads photodiode signals on the Red Pitaya's fast analog inputs
(IN1, IN2) and outputs corrective voltages on the fast analog outputs
(OUT1, OUT2) to stabilize laser intensity through an AOM or similar actuator.

```
┌───────────────────────────────────────────────────┐
│  Red Pitaya  (rp-XXXX.local)                    │
│                                                   │
│  SMA IN1 ─► PID Ch1 (RT thread) ─► SMA OUT1      │
│  SMA IN2 ─► PID Ch2 (RT thread) ─► SMA OUT2      │
│                                                   │
│  TCP server :5000  (telemetry + commands)          │
└───────────────────┬───────────────────────────────┘
                    │ TCP
┌───────────────────▼───────────────────────────────┐
│  PC  (Qt GUI)                                     │
│  Per-channel: PID on/off, gains, loop rate, plots │
└───────────────────────────────────────────────────┘
```

## Features

- **Two independent PID channels** using 14-bit, 125 MS/s fast analog I/O
- Both channels **disabled by default** -- enable from the GUI
- Adjustable PID gains (Kp, Ki, Kd), setpoint, loop rate, and error sign
- Optional AOM linearization lookup table per channel
- Configurable input/output scale and offset for analog signal conditioning
- Anti-windup integral clamping
- Real-time plots of input voltage and output voltage (pyqtgraph, ~100 Hz)
- Simple text-based TCP protocol (debuggable with `telnet`)

## Repository Structure

```
redPitaLock/
├── firmware/
│   ├── Makefile              # Build on the Red Pitaya
│   └── src/
│       ├── main.c            # Daemon: two RT PID threads + TCP server
│       ├── config.h          # Default parameters (gains, rates, scaling)
│       ├── pid.c / pid.h     # PID algorithm with anti-windup
│       ├── aom_lut.c / .h    # AOM drive linearization lookup table
│       ├── analog_io.c / .h  # librp fast analog I/O abstraction
│       └── tcp_server.c / .h # Telemetry streaming + command parser
├── scripts/
│   ├── monitor_rp.py         # PySide6 Qt control GUI
│   └── requirements.txt      # Python dependencies
├── .gitignore
└── README.md
```

## Hardware Setup

### Voltage Ranges

The Red Pitaya fast I/O operates at **+/-1 V** (LV jumper setting), which
differs from the original 0-3.3 V Pico setup. External signal conditioning
is required:

| Signal | Pico Range | RP Fast I/O Range | Recommended Conditioning |
|--------|-----------|-------------------|--------------------------|
| Photodiode input | 0-3.3 V | +/-1 V (LV) | 3.3:1 resistive voltage divider |
| AOM drive output | 0.18-1.25 V | +/-1 V | Op-amp offset + gain stage |

The firmware has per-channel `input_scale`, `input_offset`, `output_scale`,
and `output_offset` parameters adjustable at runtime via the GUI or TCP
commands.

### Connections

| Red Pitaya Port | Signal |
|-----------------|--------|
| SMA IN1 | Photodiode channel 1 (through voltage divider) |
| SMA OUT1 | AOM drive channel 1 (through conditioning circuit) |
| SMA IN2 | Photodiode channel 2 (through voltage divider) |
| SMA OUT2 | AOM drive channel 2 (through conditioning circuit) |
| Ethernet | Network connection to PC |

## Building the Firmware

The firmware is compiled **on the Red Pitaya** itself (ARM Linux with `gcc`
and `librp` pre-installed).

```bash
# From your PC -- copy the source to the Red Pitaya
scp -r firmware/ root@rp-XXXX.local:/root/redPitaLock/

# SSH into the Red Pitaya (default password: root)
ssh root@rp-XXXX.local

# Build
cd /root/redPitaLock/firmware
make

# Run (requires root for real-time thread priority)
./stabilizer_rp
```

If the Red Pitaya web apps are running and conflict with the FPGA
acquisition/generation, stop them first:

```bash
systemctl stop redpitaya_nginx
```

### Running as a Service

To start automatically on boot:

```bash
cat > /etc/systemd/system/stabilizer.service << 'EOF'
[Unit]
Description=redPitaLock PID Stabilizer
After=network.target

[Service]
ExecStart=/root/redPitaLock/firmware/stabilizer_rp
Restart=on-failure
Nice=-20

[Install]
WantedBy=multi-user.target
EOF

systemctl enable stabilizer
systemctl start stabilizer
```

## Running the GUI

On your PC:

```bash
cd scripts/
pip install -r requirements.txt
python monitor_rp.py                    # default: rp-XXXX.local:5000
python monitor_rp.py 192.168.1.100     # explicit IP
python monitor_rp.py rp-XXXX.local 5000  # explicit host + port
```

### GUI Controls (per channel)

| Control | Description |
|---------|-------------|
| **PID ON/OFF** | Toggle PID feedback (off by default) |
| **Setpoint (V)** | Target photodiode voltage |
| **Kp, Ki, Kd** | PID gains |
| **Loop period (us)** | Control loop update interval |
| **Error sign** | +1 (normal) or -1 (inverted feedback) |
| **AOM linearization** | Enable/disable the AOM drive lookup table |
| **Reset PID integrator** | Zero the integral accumulator |

## TCP Protocol

The daemon listens on port 5000. Connect with `telnet rp-XXXX.local 5000`
to debug.

### Telemetry (server -> client, ~100 Hz)

```
D <ch> <input_V> <output_V> <setpoint_V> <enabled>
```

Example:
```
D 0 1.4832 0.7521 1.5000 1
D 1 0.3210 0.0000 0.5000 0
```

### Commands (client -> server)

```
SET <ch> enabled 1              # enable PID
SET <ch> enabled 0              # disable PID
SET <ch> setpoint 1.500         # set target voltage
SET <ch> kp 4.0                 # proportional gain
SET <ch> ki 2.0                 # integral gain
SET <ch> kd 0.04                # derivative gain
SET <ch> loop_rate 1000         # loop period in microseconds
SET <ch> error_sign 1.0         # feedback polarity (+1 or -1)
SET <ch> use_lut 1              # enable AOM linearization
SET <ch> in_scale 3.3           # input calibration scale
SET <ch> in_offset 0.0          # input calibration offset
SET <ch> out_scale 1.0          # output calibration scale
SET <ch> out_offset 0.0         # output calibration offset
SET <ch> reset 0                # reset PID integrator
GET <ch> params                 # query all parameters
```

## PID Algorithm

The PID controller is identical to the original stabilizerPi implementation:

- **P term:** `Kp * error`
- **I term:** `Ki * integral`, with clamping to `+/-integral_max` and
  anti-windup (integral freezes when output saturates in the same direction
  as the error)
- **D term:** `Kd * (error - prev_error) / dt`
- **Output** clamped to `[out_min, out_max]` (default 0-1 normalized)

When the AOM LUT is enabled, the normalized 0-1 PID output is mapped through
a 13-point piecewise-linear curve to the physical AOM drive voltage
(~0.18-1.25 V).

## Origin

Ported from [stabilizerPi](https://github.com/beneaze/stabilizerPi), a
Raspberry Pi Pico-based single-channel laser intensity stabilizer.

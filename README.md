# VSSS PC Control System

PC-side control software for Very Small Size Soccer (VSSS) robots. Reads robot positions from the **FIRASim** simulator (or a vision system), computes path tracking commands using an **LQR controller**, and sends differential wheel velocities to the robots via the simulator or a serial ESP32 bridge.

```
FIRASim (UDP Multicast) -> PC Control Loop -> Serial (pyserial) -> ESP32 Bridge -> ESP-NOW -> Robots
                               ^                                                                |
                               +---------------- Serial <- ESP32 Bridge <- ESP-NOW <- Odometry -+
```

## Features

- **LQR Path Tracking** — follows reference trajectories (line, square, circle, S-curve, spline) using a Linear Quadratic Regulator with Kanayama error projection
- **Trajectory Generation** — parametric shapes with arc-length interpolation and analytic curvature computation for splines
- **Gain Scheduling** — automatic Q/R weight adaptation near field walls to avoid collisions
- **Adaptive Speed** — reduces velocity at high-curvature sections and decelerates near the end of the path
- **Dual Comm Backends** — send commands to FIRASim (UDP) or physical robots (Serial → ESP32 → ESP-NOW)
- **Plotting & Analysis** — reference trajectory visualization and actual-vs-reference tracking results on the VSSS field

## Setup

Requires **Python 3.14+** and [uv](https://docs.astral.sh/uv/).

```bash
# Install all dependencies
uv sync
```

## Commands

### Trajectory Tracking (FIRASim)

Run the LQR controller to track a trajectory shape in real time. Requires FIRASim running.

```bash
# Available shapes: line, square, circle, s_curve, spline
uv run python scripts/run_trajectory.py --shape spline

# Options
uv run python scripts/run_trajectory.py --shape circle --speed 0.4   # Custom speed (m/s)
uv run python scripts/run_trajectory.py --shape square --yellow      # Control yellow team
uv run python scripts/run_trajectory.py --shape line --id 1          # Control robot ID 1
```

Tracking results are saved to `graphs/lqr_tracking_results_<shape>.png`.

### Generate Trajectory Plots

Plot all reference trajectories on the VSSS field (no simulator needed):

```bash
uv run python scripts/plot_path.py           # Save plots to graphs/
uv run python scripts/plot_path.py --show     # Also display interactively
```

Generates: `graphs/{line,square,circle,s_curve,spline}_trajectory.png`

### Generate Protobuf Classes

Regenerate the Python protobuf bindings from the `.proto` definitions:

```bash
cd scripts && bash generate_proto.sh
```

Output goes to `vsss/vision/proto_generated/`.

### Vision Reader Test

Print live ball and robot positions from FIRASim:

```bash
uv run -m vsss.vision.reader
```

### Simulator Command Test

Send test wheel velocity commands to FIRASim:

```bash
uv run -m vsss.comms.sim_sender
```

### Serial Command Test

Send a test packet to the ESP32 bridge over USB serial:

```bash
uv run -m vsss.comms.serial_sender              # Default: /dev/ttyUSB0
uv run -m vsss.comms.serial_sender /dev/ttyACM0  # Custom port
```

### Run Tests

```bash
uv run pytest
```

## Project Structure

```
vsss-pc/
├── config.yaml                  # Robot dimensions, field size, network addresses
├── pyproject.toml               # Dependencies and project metadata
│
├── vsss/                        # Main package
│   ├── config.py                # YAML config loader
│   ├── control/
│   │   ├── base.py              # Abstract controller interface
│   │   └── lqr_tracker.py       # LQR path tracking with wall gain scheduling
│   ├── trajectory/
│   │   ├── path.py              # Arc-length interpolated path with curvature
│   │   └── shapes.py            # Shape generators (line, square, circle, s_curve, spline)
│   ├── kinematics/
│   │   └── differential.py      # Unicycle ↔ differential drive conversion
│   ├── vision/
│   │   ├── reader.py            # UDP multicast receiver for FIRASim
│   │   └── proto_generated/     # Auto-generated protobuf classes
│   ├── comms/
│   │   ├── base.py              # RobotCommand dataclass + CommandSender interface
│   │   ├── sim_sender.py        # FIRASim UDP command sender
│   │   └── serial_sender.py     # ESP32 serial bridge sender (CRC-8)
│   └── analysis/
│       └── plot_trajectory.py   # VSSS field drawing and trajectory plotting
│
├── scripts/
│   ├── run_trajectory.py        # Main LQR tracking loop (FIRASim)
│   ├── plot_path.py             # Generate reference trajectory plots
│   └── generate_proto.sh        # Protobuf code generation
│
├── tests/                       # Unit tests (pytest)
├── proto/                       # Protobuf definitions (git submodule)
└── graphs/                      # Generated plots (gitignored)
```

## Configuration

All physical and network parameters are in [`config.yaml`](config.yaml):

| Section | Key Parameters |
|---------|---------------|
| `robot_dimensions` | `wheel_radius` (0.03m), `track_width` (0.075m), max velocities |
| `field_dimensions` | `play_area_length` (1.50m), `play_area_width` (1.30m) |
| `firasim` | Multicast read IP/port, command send IP/port |
| `hardware` | ESP32 bridge MAC, robot MAC addresses |
| `mode` | `"sim"` (FIRASim) or `"hw"` (physical robots) |

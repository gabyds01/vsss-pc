# VSSS PC Control System

## Project Goal

The final goal of this project is to build a complete closed-loop control system for Very Small Size Soccer (VSSS) robots. The software on the PC acts as the brain: it reads robot and ball positions from a simulator (FIRASim) or a vision system, runs path tracking controllers to calculate target trajectories, and sends target differential wheel velocities (in meters per second) to an ESP32 serial-to-wireless bridge, which routes them to the physical robots. The system also processes telemetry and encoder feedback returned by the robots to monitor and plot actual performance against target trajectories on a web-based dashboard.

```
FIRASim (UDP Multicast) -> PC Control Loop -> Serial (pyserial) -> ESP32 Bridge -> ESP-NOW -> Robots
                               ^                                                                |
                               +---------------- Serial <- ESP32 Bridge <- ESP-NOW <- Odometry -+
```

## Current State

The foundation of the system is established with the following implemented components:

### Dependency and Package Management
* Configured dependency tracking using uv. Project dependencies include protobuf, pyserial, numpy, scipy, and websockets.
* Integrated the generated Python protobuf classes under the vsss package, with automatic system path resolution enabled on package import.

### Configuration
* Configured config.yaml to load central parameters for FIRASim network addresses and robot dimensions.

### Vision and State Reception
* Implemented the UDPReceiver class in vsss/vision/reader.py. It binds to the FIRASim multicast group, reads incoming environment states, and parses them into frame data, including ball and robot locations.

### Communication and Command Transmission
* Implemented the RobotCommand dataclass and CommandSender abstract base interface in vsss/comms/base.py.
* Implemented the SimSender in vsss/comms/sim_sender.py to serialize and send command packets directly to the simulator.
* Implemented the SerialSender in vsss/comms/serial_sender.py to communicate with the physical ESP32 bridge over serial. It packages wheel velocities, scales them to the required range, and appends a Dallas/Maxim CRC-8 checksum.

## Setup and Run

Install dependencies using uv:
```bash
uv sync
```

### Running the Vision Reader Test
To start the UDP receiver and print positions of the ball and robots broadcasted by FIRASim:
```bash
uv run -m vsss.vision.reader
```

### Running the Simulator Command Test
To send test command packets to a local FIRASim instance:
```bash
uv run -m vsss.comms.sim_sender
```

### Running the Serial Command Test
To test sending a packet over the USB serial interface to the ESP32 bridge:
```bash
uv run -m vsss.comms.serial_sender [optional_port_name]
```
By default, the serial sender targets /dev/ttyUSB0 unless a different port is specified as an argument.

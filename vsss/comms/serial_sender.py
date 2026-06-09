import struct
import serial
from vsss.config import SETTINGS
from vsss.comms.base import CommandSender, RobotCommand

# Check if serial configuration is defined in config.yaml, otherwise use defaults
serial_config = SETTINGS.get("serial", {}) if SETTINGS else {}
DEFAULT_PORT = serial_config.get("port", "/dev/ttyUSB0")
DEFAULT_BAUD = serial_config.get("baudrate", 115200)

PACKET_HEADER = 0xAA
PACKET_LENGTH = 0x08
SCALE_FACTOR = 100


def crc8_dallas(data: bytes) -> int:
    """Compute CRC-8 Dallas/Maxim — matches the firmware implementation."""
    crc = 0x00
    for byte in data:
        inbyte = byte
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc


def build_packet(cmd: RobotCommand) -> bytes:
    """Build a single 8-byte binary packet from a RobotCommand."""
    wl = int(max(-32767, min(32767, cmd.wheel_left * SCALE_FACTOR)))
    wr = int(max(-32767, min(32767, cmd.wheel_right * SCALE_FACTOR)))

    payload = struct.pack("<BBBhh", PACKET_HEADER, PACKET_LENGTH, cmd.robot_id, wl, wr)
    crc = crc8_dallas(payload)
    return payload + bytes([crc])


class SerialSender(CommandSender):
    """Sends RobotCommand packets over USB serial to the ESP32 bridge."""

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUD):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.rx_buffer = bytearray()

    def open(self):
        """Open the serial connection."""
        if self.ser and self.ser.is_open:
            return

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)

    def close(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send(self, commands: list[RobotCommand]):
        """Send a list of RobotCommands as binary packets."""
        if not self.is_open:
            try:
                self.open()
            except Exception:
                # Silently fail if unable to open port to avoid crashing main loop
                return

        if not self.is_open:
            return

        for cmd in commands:
            packet = build_packet(cmd)
            self.ser.write(packet)

    def read_telemetry(self) -> list[dict]:
        """Read all available bytes from serial, update the buffer, and parse valid telemetry packets."""
        if not self.is_open:
            try:
                self.open()
            except Exception:
                return []

        if not self.is_open:
            return []

        try:
            if self.ser.in_waiting > 0:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))
        except Exception:
            return []

        packets = []
        while True:
            idx = self.rx_buffer.find(0xBB)
            if idx == -1:
                self.rx_buffer.clear()
                break

            if idx > 0:
                del self.rx_buffer[:idx]

            if len(self.rx_buffer) < 2:
                break

            pkt_len = self.rx_buffer[1]
            if pkt_len != 0x0A:
                del self.rx_buffer[0]
                continue

            expected_size = 2 + pkt_len
            if len(self.rx_buffer) < expected_size:
                break

            packet_bytes = bytes(self.rx_buffer[:expected_size])
            payload = packet_bytes[:-1]
            expected_crc = packet_bytes[-1]

            if crc8_dallas(payload) == expected_crc:
                robot_id = packet_bytes[2]
                wl, wr, ed_l, ed_r = struct.unpack("<hhhh", packet_bytes[3:11])

                packets.append({
                    "robot_id": robot_id,
                    "wheel_left_vel": wl / SCALE_FACTOR,
                    "wheel_right_vel": wr / SCALE_FACTOR,
                    "enc_delta_left": ed_l,
                    "enc_delta_right": ed_r,
                })
                del self.rx_buffer[:expected_size]
            else:
                del self.rx_buffer[0]

        return packets

    def __del__(self):
        self.close()


if __name__ == "__main__":
    import sys

    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]

    sender = SerialSender(port=port, baudrate=DEFAULT_BAUD)
    print(f"Attempting to open serial port {port}...")
    try:
        sender.open()
        print("Serial port opened successfully.")
        print(
            "Enter left and right velocities in m/s (e.g., '0.15 -0.15' or '0 0' to stop)."
        )
        print("Type 'q' or Ctrl+C to exit.")
        print(
            "-------------------------------------------------------------------------"
        )

        while True:
            try:
                user_input = input("Enter velocities (left right): ").strip()
                if not user_input:
                    continue
                if user_input.lower() == "q":
                    break

                parts = user_input.split()
                if len(parts) != 2:
                    print(
                        "Error: Please enter exactly two numbers separated by a space."
                    )
                    continue

                wl = float(parts[0])
                wr = float(parts[1])

                cmd = RobotCommand(
                    robot_id=0, yellow_team=False, wheel_left=wl, wheel_right=wr
                )
                print(
                    f"Sending to Robot 0: wheel_left={wl:.3f} m/s, wheel_right={wr:.3f} m/s"
                )
                sender.send([cmd])
            except ValueError:
                print("Error: Input must be numeric.")
            except (KeyboardInterrupt, EOFError):
                print("\nStopping robot and exiting...")
                stop_cmd = RobotCommand(
                    robot_id=0, yellow_team=False, wheel_left=0.0, wheel_right=0.0
                )
                sender.send([stop_cmd])
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sender.close()

import socket
from vsss.config import SETTINGS
from vsss.comms.base import CommandSender, RobotCommand
from vsss.vision.proto_generated.packet_pb2 import Packet

# Default target destination for FIRASim commands
MCAST_IP = SETTINGS["firasim"]["send_ip"]
MCAST_PORT = SETTINGS["firasim"]["send_port"]
WHEEL_RADIUS = SETTINGS["robot_dimensions"]["wheel_radius"]


class SimSender(CommandSender):
    """Sends velocity commands to FIRASim via UDP protobuf packets."""

    def __init__(self):
        self.sock = None
        self.open()

    def open(self):
        """Open the UDP socket."""
        if self.sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
            )
            self.sock.setsockopt(
                socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1
            )  # Local subnet

    def close(self):
        """Close the UDP socket."""
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send(self, commands: list[RobotCommand]):
        """Send a list of robot commands to the simulator."""
        if self.sock is None:
            self.open()
        data = self.serialize(commands)
        self.sock.sendto(data, (MCAST_IP, MCAST_PORT))

    def serialize(self, commands: list[RobotCommand]) -> bytes:
        """Serialize list of RobotCommands to a binary Protobuf string."""
        packet = Packet()

        for robot_command in commands:
            cmd = packet.cmd.robot_commands.add()
            cmd.id = robot_command.robot_id
            cmd.yellowteam = robot_command.yellow_team
            # Convert wheel linear velocity (m/s) to wheel angular velocity (rad/s) for FIRASim
            cmd.wheel_left = robot_command.wheel_left / WHEEL_RADIUS
            cmd.wheel_right = robot_command.wheel_right / WHEEL_RADIUS

        return packet.SerializeToString()


if __name__ == "__main__":
    sender = SimSender()
    print(f"Sending test commands to FIRASim on {MCAST_IP}:{MCAST_PORT}...")

    # Send test commands for blue robot 0 and yellow robot 0
    cmd_blue = RobotCommand(
        robot_id=0, yellow_team=False, wheel_left=0.0, wheel_right=-0.0
    )
    cmd_yellow = RobotCommand(
        robot_id=0, yellow_team=True, wheel_left=-0.0, wheel_right=0.0
    )

    sender.send([cmd_blue, cmd_yellow])
    print("Test commands sent successfully.")

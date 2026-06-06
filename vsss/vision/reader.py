import socket
import struct
import asyncio
from vsss.config import SETTINGS
from vsss.vision.proto_generated.packet_pb2 import Environment

MCAST_IP = SETTINGS["firasim"]["read_ip"]
MCAST_PORT = SETTINGS["firasim"]["read_port"]


class UDPReceiver:
    """UDP multicast receiver for FIRASim state updates."""

    def __init__(self):
        # Set up socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Set to non-blocking mode
        self.sock.setblocking(False)

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Join group with port
        self.mreq = struct.pack("4sl", socket.inet_aton(MCAST_IP), socket.INADDR_ANY)
        self.sock.bind(("", MCAST_PORT))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, self.mreq)

        self.is_connected = False

    async def receive(self) -> bytes:
        """Receive a raw packet from the multicast socket asynchronously."""
        loop = asyncio.get_running_loop()
        data = await loop.sock_recv(self.sock, 65536)
        return data

    def deserialize(self, data: bytes):
        """Parse raw bytes into FIRASim Environment message."""
        env = Environment()
        env.ParseFromString(data)
        return env.frame, env.field, env.goals_blue, env.goals_yellow


async def main():
    print(f"Starting UDPReceiver on {MCAST_IP}:{MCAST_PORT}...")
    receiver = UDPReceiver()
    while True:
        try:
            data = await receiver.receive()
            frame, field, goals_blue, goals_yellow = receiver.deserialize(data)
            print(
                f"Received frame. Ball at: x={frame.ball.x:.2f}, y={frame.ball.y:.2f}"
            )
            if frame.robots_blue:
                for r in frame.robots_blue:
                    print(
                        f"  Blue Robot {r.robot_id}: x={r.x:.2f}, y={r.y:.2f}, orientation={r.orientation:.2f}"
                    )
            if frame.robots_yellow:
                for r in frame.robots_yellow:
                    print(
                        f"  Yellow Robot {r.robot_id}: x={r.x:.2f}, y={r.y:.2f}, orientation={r.orientation:.2f}"
                    )
        except Exception as e:
            print(f"Error: {e}")
            await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")

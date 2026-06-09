from vsss.comms.base import CommandSender, RobotCommand
from vsss.comms.sim_sender import SimSender
from vsss.comms.serial_sender import SerialSender


class BothSender(CommandSender):
    """Sends velocity commands to both FIRASim (simulation) and ESP32 bridge (serial) simultaneously."""

    def __init__(self, port: str = None, baudrate: int = None):
        self.sim_sender = SimSender()
        kwargs = {}
        if port is not None:
            kwargs["port"] = port
        if baudrate is not None:
            kwargs["baudrate"] = baudrate
        self.serial_sender = SerialSender(**kwargs)

    def open(self):
        """Open both simulation and serial communication interfaces."""
        self.sim_sender.open()
        self.serial_sender.open()

    def close(self):
        """Close both simulation and serial communication interfaces."""
        self.sim_sender.close()
        self.serial_sender.close()

    def send(self, commands: list[RobotCommand]):
        """Send a list of robot commands to both senders."""
        self.sim_sender.send(commands)
        self.serial_sender.send(commands)

    def read_telemetry(self) -> list[dict]:
        """Read telemetry from the serial connection."""
        return self.serial_sender.read_telemetry()

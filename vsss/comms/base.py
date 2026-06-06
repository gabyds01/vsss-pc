from dataclasses import dataclass
from abc import ABC, abstractmethod


@dataclass
class RobotCommand:
    robot_id: int
    yellow_team: bool
    wheel_left: float
    wheel_right: float


class CommandSender(ABC):
    """Abstract base class for sending commands to the robots (simulation or hardware)."""

    @abstractmethod
    def open(self):
        """Open the communication interface."""
        pass

    @abstractmethod
    def close(self):
        """Close the communication interface."""
        pass

    @abstractmethod
    def send(self, commands: list[RobotCommand]):
        """Send a list of robot commands."""
        pass

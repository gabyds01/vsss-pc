from abc import ABC, abstractmethod


class BaseController(ABC):
    """Abstract base class for all path tracking controllers."""

    @abstractmethod
    def compute_commands(
        self,
        current_state: tuple[float, float, float],
        ref_state: tuple[float, float, float],
        ref_commands: tuple[float, float],
    ) -> tuple[float, float]:
        """Compute the control commands (v, omega) for the robot.

        Args:
            current_state (tuple): Current robot state (x, y, theta)
            ref_state (tuple): Target/reference state (x_r, y_r, theta_r)
            ref_commands (tuple): Reference feedforward control inputs (v_r, omega_r)

        Returns:
            v (float): Target linear velocity in m/s
            omega (float): Target angular velocity in rad/s
        """
        pass

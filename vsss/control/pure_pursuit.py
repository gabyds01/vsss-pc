import numpy as np
from vsss.config import SETTINGS
from vsss.control.base import BaseController
from vsss.kinematics.differential import unicycle_to_differential

# Load physical field parameters
PLAY_AREA_LENGTH = SETTINGS["field_dimensions"]["play_area_length"]
PLAY_AREA_WIDTH = SETTINGS["field_dimensions"]["play_area_width"]
TRACK_WIDTH = SETTINGS["robot_dimensions"]["track_width"]

X_MIN, X_MAX = -PLAY_AREA_LENGTH / 2.0, PLAY_AREA_LENGTH / 2.0
Y_MIN, Y_MAX = -PLAY_AREA_WIDTH / 2.0, PLAY_AREA_WIDTH / 2.0


class PurePursuitTracker(BaseController):
    """Pure Pursuit geometric path tracking controller.

    Steers the robot toward a lookahead point on the reference path by fitting
    a circular arc from the robot's current pose to that point.  The curvature
    of the arc determines the angular velocity command.

    Key formula (in robot-local frame):
        κ = 2 · y_local / L_d²
        ω = v · κ
    """

    def __init__(
        self,
        lookahead_distance: float = 0.12,
        min_lookahead: float = 0.05,
        margin_target: float = 0.08,
    ):
        """
        Args:
            lookahead_distance: Arc-length distance ahead of the closest point
                used to pick the target on the path (meters).
            min_lookahead: Floor for the Euclidean distance to the lookahead
                point, preventing division-by-zero when very close (meters).
            margin_target: Safety margin for clamping the target inside the
                play area (meters).
        """
        self.lookahead_distance = lookahead_distance
        self.min_lookahead = min_lookahead
        self.margin_target = margin_target

    def compute_commands(
        self,
        current_state: tuple[float, float, float],
        ref_state: tuple[float, float, float],
        ref_commands: tuple[float, float],
    ) -> tuple[float, float]:
        """Compute target linear and angular velocities (v, omega) using Pure Pursuit.

        Args:
            current_state (tuple): Current robot state (x, y, theta)
            ref_state (tuple): Lookahead target point on the path (x_r, y_r, theta_r).
                Only x_r and y_r are used; theta_r is ignored.
            ref_commands (tuple): Reference inputs (v_r, omega_r).
                Only v_r is used; omega_r is ignored.

        Returns:
            v (float): Target linear velocity (m/s)
            omega (float): Target angular velocity (rad/s)
        """
        x, y, theta = current_state
        x_r, y_r, _ = ref_state
        v_r, _ = ref_commands

        # Clamp target inside the play area
        x_r = np.clip(x_r, X_MIN + self.margin_target, X_MAX - self.margin_target)
        y_r = np.clip(y_r, Y_MIN + self.margin_target, Y_MAX - self.margin_target)

        # Vector from robot to lookahead point (global frame)
        dx = x_r - x
        dy = y_r - y

        # Transform to robot-local frame
        x_local = dx * np.cos(theta) + dy * np.sin(theta)
        y_local = -dx * np.sin(theta) + dy * np.cos(theta)

        # Euclidean distance to lookahead point (floor to prevent div/0)
        L_d = max(np.hypot(x_local, y_local), self.min_lookahead)

        # Pure Pursuit curvature: κ = 2 · y_local / L_d²
        kappa = 2.0 * y_local / (L_d**2)

        # If the target is behind the robot, pivot in place toward it
        if x_local < 0:
            v = 0.0
            omega = np.sign(y_local or 1.0) * abs(v_r) * 3.0
        else:
            v = v_r
            omega = v * kappa

        return v, omega

    def compute_wheel_velocities(
        self,
        current_state: tuple[float, float, float],
        ref_state: tuple[float, float, float],
        ref_commands: tuple[float, float],
    ) -> tuple[float, float]:
        """Compute the left and right wheel velocities (v_left, v_right) in m/s.

        Useful for direct routing to FIRASim or serial transmission.
        """
        v, omega = self.compute_commands(current_state, ref_state, ref_commands)
        return unicycle_to_differential(v, omega, TRACK_WIDTH)

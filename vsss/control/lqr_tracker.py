import numpy as np
from scipy.linalg import solve_continuous_are
from vsss.config import SETTINGS
from vsss.control.base import BaseController
from vsss.kinematics.differential import unicycle_to_differential

# Load physical field parameters
PLAY_AREA_LENGTH = SETTINGS["field_dimensions"]["play_area_length"]
PLAY_AREA_WIDTH = SETTINGS["field_dimensions"]["play_area_width"]
WHEEL_RADIUS = SETTINGS["robot_dimensions"]["wheel_radius"]
TRACK_WIDTH = SETTINGS["robot_dimensions"]["track_width"]

X_MIN, X_MAX = -PLAY_AREA_LENGTH / 2.0, PLAY_AREA_LENGTH / 2.0
Y_MIN, Y_MAX = -PLAY_AREA_WIDTH / 2.0, PLAY_AREA_WIDTH / 2.0


class LQRTracker(BaseController):
    """Linear Quadratic Regulator (LQR) path tracking controller with Gain Scheduling for wall evasion."""

    def __init__(
        self,
        q_x: float = 15.0,
        q_y: float = 20.0,
        q_theta: float = 2.0,
        r_v: float = 1.0,
        r_omega: float = 0.5,
        d_safe: float = 0.16,      # 16 cm safety zone (2x robot size)
        beta_q: float = 40.0,      # Angular penalty multiplier near walls
        gamma_r: float = 8.0,      # Angular actuator cost divisor near walls
        margin_target: float = 0.08,  # Target clamping margin (8 cm)
    ):
        # Weight matrices coefficients
        self.q_x = q_x
        self.q_y = q_y
        self.q_theta_nominal = q_theta
        self.r_v = r_v
        self.r_omega_nominal = r_omega

        # Safety & Gain scheduling parameters
        self.d_safe = d_safe
        self.beta_q = beta_q
        self.gamma_r = gamma_r
        self.margin_target = margin_target

        # Preset B matrix (continuous time error dynamics)
        # e_dot = A*e + B*u
        self.B = np.array([[-1.0, 0.0],
                           [0.0, 0.0],
                           [0.0, -1.0]])

    def compute_commands(
        self,
        current_state: tuple[float, float, float],
        ref_state: tuple[float, float, float],
        ref_commands: tuple[float, float],
    ) -> tuple[float, float]:
        """Compute the target linear and angular velocities (v, omega) using LQR.

        Args:
            current_state (tuple): Current robot state (x, y, theta)
            ref_state (tuple): Desired target state (x_r, y_r, theta_r)
            ref_commands (tuple): Reference feedforward inputs (v_r, omega_r)

        Returns:
            v (float): Target linear velocity (m/s)
            omega (float): Target angular velocity (rad/s)
        """
        x, y, theta = current_state
        x_r, y_r, theta_r = ref_state
        v_r, omega_r = ref_commands

        # 1. Clamping Target coordinates to keep them safely within the play field
        x_r_safe = np.clip(x_r, X_MIN + self.margin_target, X_MAX - self.margin_target)
        y_r_safe = np.clip(y_r, Y_MIN + self.margin_target, Y_MAX - self.margin_target)

        # 2. Wall Proximity Measurement
        d_wall = min(x - X_MIN, X_MAX - x, y - Y_MIN, Y_MAX - y)

        # 3. Repulsion factor (between 0.0 and 1.0)
        eta = max(0.0, min(1.0, (self.d_safe - d_wall) / self.d_safe))

        # 4. Dynamic Gain Scheduling
        # Attenuate forward reference speed near walls to allow pivot rotation
        v_r_scaled = v_r * (1.0 - eta)
        
        # Amplify angular penalty (Increase Q penalty for heading error)
        q_theta = self.q_theta_nominal * (1.0 + self.beta_q * eta)
        
        # Reduce angular control cost (Make angular action cheaper in R matrix)
        r_omega = self.r_omega_nominal / (1.0 + self.gamma_r * eta)

        # 5. Assemble state-space matrices A, Q, and R
        # Use a tiny sign-preserving value for v_r_scaled to keep the pair (A, B) controllable
        v_r_calc = v_r_scaled if abs(v_r_scaled) > 0.01 else 0.01 * np.sign(v_r_scaled or 1.0)
        
        A = np.array([[0.0, omega_r, 0.0],
                      [-omega_r, 0.0, v_r_calc],
                      [0.0, 0.0, 0.0]])

        Q = np.diag([self.q_x, self.q_y, q_theta])
        R = np.diag([self.r_v, r_omega])

        # 6. Solve Continuous Algebraic Riccati Equation (ARE)
        try:
            P = solve_continuous_are(A, self.B, Q, R)
            K = np.linalg.inv(R) @ self.B.T @ P
        except Exception:
            # Fallback proportional gains if matrix solving fails (e.g. system is close to singular)
            K = np.array([[2.0, 0.0, 0.0],
                          [0.0, 2.5, 1.5]])

        # 7. Kanayama local error projection
        dx = x_r_safe - x
        dy = y_r_safe - y
        dtheta = (theta_r - theta + np.pi) % (2.0 * np.pi) - np.pi

        x_e = dx * np.cos(theta) + dy * np.sin(theta)
        y_e = -dx * np.sin(theta) + dy * np.cos(theta)
        theta_e = dtheta

        e = np.array([x_e, y_e, theta_e])

        # 8. Compute LQR Control Law: u = u_r + delta_u = [v_r_scaled, omega_r]^T - K * e
        delta_u = -K @ e
        v = v_r_scaled + delta_u[0]
        omega = omega_r + delta_u[1]

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

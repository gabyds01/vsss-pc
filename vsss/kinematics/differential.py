from vsss.config import SETTINGS

# Load default physical parameters from configuration
WHEEL_RADIUS = SETTINGS["robot_dimensions"]["wheel_radius"]
TRACK_WIDTH = SETTINGS["robot_dimensions"]["track_width"]

# Determine maximum velocity limit based on the system execution mode ("sim", "hw", or "both")
MODE = SETTINGS.get("mode", "sim")
if MODE == "sim":
    MAX_VELOCITY = SETTINGS["robot_dimensions"].get("max_velocity_sim", 1.5)
else:
    MAX_VELOCITY = SETTINGS["robot_dimensions"].get("max_velocity_hw", 0.34)


def saturate_velocities(
    v_left: float, v_right: float, max_vel: float = None
) -> tuple[float, float]:
    """Saturate wheel velocities to the maximum limit, scaling both proportionally

    to preserve the steering ratio/curvature of the robot path.
    """
    if max_vel is None:
        max_vel = MAX_VELOCITY
    max_val = max(abs(v_left), abs(v_right))
    if max_val > max_vel:
        v_left = (v_left / max_val) * max_vel
        v_right = (v_right / max_val) * max_vel
    return v_left, v_right


def unicycle_to_differential(
    v: float, omega: float, track_width: float = TRACK_WIDTH, saturate: bool = True
) -> tuple[float, float]:
    """Convert unicycle model commands (linear velocity v and angular velocity omega)

    to differential drive wheel velocities (v_left, v_right) in m/s.

    Formula:
    v_left = v - (omega * track_width) / 2
    v_right = v + (omega * track_width) / 2
    """
    v_left = v - (omega * track_width) / 2.0
    v_right = v + (omega * track_width) / 2.0

    if saturate:
        v_left, v_right = saturate_velocities(v_left, v_right)

    return v_left, v_right


def differential_to_unicycle(
    v_left: float, v_right: float, track_width: float = TRACK_WIDTH
) -> tuple[float, float]:
    """Convert differential drive wheel velocities (v_left, v_right)

    to unicycle model velocities (linear velocity v and angular velocity omega).

    Formula:
    v = (v_left + v_right) / 2
    omega = (v_right - v_left) / track_width
    """
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / track_width
    return v, omega

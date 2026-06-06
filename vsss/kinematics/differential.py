from vsss.config import SETTINGS

# Load default physical parameters from configuration
WHEEL_RADIUS = SETTINGS["robot_dimensions"]["wheel_radius"]
TRACK_WIDTH = SETTINGS["robot_dimensions"]["track_width"]


def unicycle_to_differential(
    v: float, omega: float, track_width: float = TRACK_WIDTH
) -> tuple[float, float]:
    """Convert unicycle model commands (linear velocity v and angular velocity omega)

    to differential drive wheel velocities (v_left, v_right) in m/s.

    Formula:
    v_left = v - (omega * track_width) / 2
    v_right = v + (omega * track_width) / 2
    """
    v_left = v - (omega * track_width) / 2.0
    v_right = v + (omega * track_width) / 2.0
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

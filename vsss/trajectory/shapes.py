import numpy as np
from .path import Path


def generate_line(
    start: tuple[float, float], end: tuple[float, float], resolution: float = 0.01
) -> Path:
    """Generates a straight line path from start to end."""
    return Path([start, end], resolution=resolution)


def generate_square(
    side: float, center: tuple[float, float] = (0.0, 0.0), resolution: float = 0.01
) -> Path:
    """Generates a square path centered at the given coordinates."""
    cx, cy = center
    half = side / 2.0
    waypoints = [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),  # Close the loop
    ]
    return Path(waypoints, resolution=resolution)


def generate_circle(
    radius: float, center: tuple[float, float] = (0.0, 0.0), resolution: float = 0.01
) -> Path:
    """Generates a circular path centered at the given coordinates."""
    cx, cy = center
    circumference = 2.0 * np.pi * radius
    num_waypoints = max(8, int(np.ceil(circumference / 0.05)))

    angles = np.linspace(0, 2.0 * np.pi, num_waypoints)
    waypoints = [(cx + radius * np.cos(a), cy + radius * np.sin(a)) for a in angles]
    return Path(waypoints, resolution=resolution)


def generate_s_curve(
    start_x: float,
    end_x: float,
    amplitude: float = 0.3,
    center_y: float = 0.0,
    resolution: float = 0.01,
) -> Path:
    """Generates a smooth S-curve path along the X axis."""
    x_vals = np.linspace(start_x, end_x, 15)
    y_vals = center_y + amplitude * np.sin(
        2.0 * np.pi * (x_vals - start_x) / (end_x - start_x)
    )
    waypoints = list(zip(x_vals, y_vals))
    return Path(waypoints, resolution=resolution)


def generate_spline(
    control_points: list[tuple[float, float]], resolution: float = 0.01
) -> Path:
    """Generates a smooth spline path passing through the given control points."""
    from scipy.interpolate import splprep, splev

    if len(control_points) < 2:
        raise ValueError("At least 2 control points are required for a spline.")

    pts = np.array(control_points)
    x = pts[:, 0]
    y = pts[:, 1]

    # Use degree 3 (cubic) unless there are not enough control points
    k = min(3, len(control_points) - 1)
    
    # Parametric spline representation, s=0 forces the spline through the control points
    tck, u = splprep([x, y], k=k, s=0)

    # Estimate length to determine density of evaluation points
    dx = np.diff(x)
    dy = np.diff(y)
    est_length = np.sum(np.sqrt(dx**2 + dy**2))
    num_points = max(100, int(np.ceil(est_length / (resolution / 10.0))))

    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine = splev(u_fine, tck)

    # Compute analytic curvature from parametric spline derivatives
    # κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
    dx_du, dy_du = splev(u_fine, tck, der=1)
    d2x_du2, d2y_du2 = splev(u_fine, tck, der=2)
    numerator = dx_du * d2y_du2 - dy_du * d2x_du2
    denominator = (dx_du**2 + dy_du**2) ** 1.5
    curvature = np.where(denominator > 1e-10, numerator / denominator, 0.0)

    waypoints = list(zip(x_fine, y_fine))
    return Path(waypoints, resolution=resolution, curvature=curvature)


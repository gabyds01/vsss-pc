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
        np.pi * (x_vals - start_x) / (end_x - start_x)
    )
    waypoints = list(zip(x_vals, y_vals))
    return Path(waypoints, resolution=resolution)

import pytest
import numpy as np
from vsss.trajectory.path import Path
from vsss.trajectory.shapes import generate_line, generate_square, generate_circle


def test_path_interpolation():
    waypoints = [(0.0, 0.0), (1.0, 0.0)]
    path = Path(waypoints, resolution=0.1)

    # Check total length
    assert path.total_length == 1.0

    # Check that interpolated points cover the start and end
    assert np.isclose(path.x[0], 0.0)
    assert np.isclose(path.x[-1], 1.0)

    # Check interpolation points resolution (10 steps of 0.1m + 1 start point = 11 points)
    assert len(path.s) >= 11


def test_closest_point_matching():
    path = generate_line(start=(0.0, 0.0), end=(1.0, 0.0), resolution=0.01)

    # Check closest point to (0.5, 0.1) -> should be on path at (0.5, 0.0) with distance 0.1
    x, y, theta, s, dist = path.get_closest_point(0.5, 0.1)
    assert np.isclose(x, 0.5, atol=0.02)
    assert np.isclose(y, 0.0, atol=0.02)
    assert np.isclose(theta, 0.0, atol=0.01)
    assert np.isclose(dist, 0.1, atol=0.01)
    assert np.isclose(s, 0.5, atol=0.02)


def test_get_point_at_distance():
    path = generate_line(start=(0.0, 0.0), end=(0.0, 1.0), resolution=0.01)

    # Get point at s = 0.5 (halfway up y-axis)
    x, y, theta = path.get_point_at_distance(0.5)
    assert np.isclose(x, 0.0, atol=0.01)
    assert np.isclose(y, 0.5, atol=0.01)
    assert np.isclose(theta, np.pi / 2.0, atol=0.01)

import numpy as np
from vsss.kinematics.differential import unicycle_to_differential, differential_to_unicycle, saturate_velocities


def test_unicycle_to_differential_straight():
    # Driving straight (v = 1.0 m/s, omega = 0.0 rad/s)
    v_left, v_right = unicycle_to_differential(1.0, 0.0, track_width=0.075)
    assert v_left == 1.0
    assert v_right == 1.0


def test_unicycle_to_differential_turn():
    # Pure spin (v = 0.0 m/s, omega = 2.0 rad/s)
    # track_width = 0.1
    # v_left = 0 - (2.0 * 0.1)/2 = -0.1
    # v_right = 0 + (2.0 * 0.1)/2 = 0.1
    v_left, v_right = unicycle_to_differential(0.0, 2.0, track_width=0.1)
    assert v_left == -0.1
    assert v_right == 0.1


def test_differential_to_unicycle_straight():
    v, omega = differential_to_unicycle(1.0, 1.0, track_width=0.075)
    assert v == 1.0
    assert omega == 0.0


def test_differential_to_unicycle_turn():
    v, omega = differential_to_unicycle(-0.1, 0.1, track_width=0.1)
    assert v == 0.0
    assert omega == 2.0


def test_proportional_saturation():
    # If target velocities are L=3.0, R=1.0 and max is 1.5,
    # it should scale them proportionally to L=1.5, R=0.5
    v_left, v_right = saturate_velocities(3.0, 1.0, max_vel=1.5)
    assert np.isclose(v_left, 1.5)
    assert np.isclose(v_right, 0.5)


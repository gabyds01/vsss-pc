import numpy as np
from vsss.control.lqr_tracker import LQRTracker


def test_lqr_initialization():
    tracker = LQRTracker()
    assert tracker.q_x == 15.0
    assert tracker.q_y == 20.0


def test_lqr_compute_commands_zero_error():
    tracker = LQRTracker()
    # When starting exactly on reference (0, 0, 0) with target (0, 0, 0) and ref commands (0.5, 0.0)
    v, omega = tracker.compute_commands(
        current_state=(0.0, 0.0, 0.0),
        ref_state=(0.0, 0.0, 0.0),
        ref_commands=(0.5, 0.0),
    )
    # Linear velocity should equal reference, angular should be zero
    assert np.isclose(v, 0.5, atol=0.02)
    assert np.isclose(omega, 0.0, atol=0.01)


def test_lqr_wall_evasion():
    tracker = LQRTracker(d_safe=0.2)
    # Place robot close to the left wall (X_MIN = -0.75m)
    # at x = -0.73m -> d_wall = 0.02m, which is in the safety zone (d_safe = 0.2m)
    # The linear velocity should be attenuated from its nominal 0.5m/s reference
    v, omega = tracker.compute_commands(
        current_state=(-0.73, 0.0, 0.0),
        ref_state=(-0.73, 0.0, 0.0),
        ref_commands=(0.5, 0.0),
    )
    assert v < 0.5
    assert v >= 0.0


def test_compute_wheel_velocities():
    tracker = LQRTracker()
    v_left, v_right = tracker.compute_wheel_velocities(
        current_state=(0.0, 0.0, 0.0),
        ref_state=(0.0, 0.0, 0.0),
        ref_commands=(0.5, 0.0),
    )
    # For linear velocity 0.5 m/s and angular velocity 0 rad/s, both wheels should spin at 0.5 m/s
    assert np.isclose(v_left, 0.5, atol=0.02)
    assert np.isclose(v_right, 0.5, atol=0.02)

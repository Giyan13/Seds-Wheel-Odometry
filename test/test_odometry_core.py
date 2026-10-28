import math

import pytest

from wheel_odometry.odometry_core import (
    WheelOdometry, normalize_angle, yaw_to_quaternion)

R = 0.075
L = 0.34


def make(alpha=1.0):
    return WheelOdometry(R, L, alpha)


def test_first_message_only_sets_baseline():
    odom = make()
    s = odom.update(123.4, 567.8, 0.0)          # big starting joint angles
    assert (s.x, s.y, s.theta, s.v, s.omega) == (0.0, 0.0, 0.0, 0.0, 0.0)
    s = odom.update(123.4, 567.8, 0.05)         # no motion afterwards
    assert (s.x, s.y, s.theta) == (0.0, 0.0, 0.0)


def test_straight_driving():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    phi = 0.0
    for _ in range(100):
        phi += 0.2                               # both wheels turn 0.2 rad per step
        s = odom.update(phi, phi, 0.1)
    assert s.x == pytest.approx(R * 20.0, rel=1e-9)
    assert s.y == pytest.approx(0.0, abs=1e-9)
    assert s.theta == pytest.approx(0.0, abs=1e-9)
    assert s.v == pytest.approx(R * 0.2 / 0.1, rel=1e-9)
    assert s.omega == pytest.approx(0.0, abs=1e-9)


def test_in_place_turn():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    left = right = 0.0
    for _ in range(50):
        left -= 0.1
        right += 0.1
        s = odom.update(left, right, 0.1)
    expected = normalize_angle(R * 10.0 / L)     # (d_r - d_l)/L, summed
    assert s.x == pytest.approx(0.0, abs=1e-9)
    assert s.y == pytest.approx(0.0, abs=1e-9)
    assert s.theta == pytest.approx(expected, abs=1e-9)
    assert s.v == pytest.approx(0.0, abs=1e-9)
    assert s.omega == pytest.approx(R * 0.2 / L / 0.1, rel=1e-9)


def test_full_rotation_returns_to_start():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    phi_total = math.pi * L / R                  # each wheel travels pi*L for 2*pi turn
    steps = 1000
    left = right = 0.0
    for _ in range(steps):
        left -= phi_total / steps
        right += phi_total / steps
        s = odom.update(left, right, 0.01)
    assert abs(normalize_angle(s.theta)) < 1e-6
    assert abs(s.x) < 1e-9 and abs(s.y) < 1e-9


def test_quarter_circle_arc():
    """Outer wheel travels further: rover should curve left and end up at (R_c, R_c)."""
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    radius_c = 1.0                               # centre-line turn radius, m
    angle = math.pi / 2
    d_l = (radius_c - L / 2) * angle
    d_r = (radius_c + L / 2) * angle
    steps = 2000
    left = right = 0.0
    for _ in range(steps):
        left += d_l / R / steps
        right += d_r / R / steps
        s = odom.update(left, right, 0.01)
    assert s.x == pytest.approx(radius_c, abs=1e-3)
    assert s.y == pytest.approx(radius_c, abs=1e-3)
    assert s.theta == pytest.approx(math.pi / 2, abs=1e-6)


def test_smooth_no_jumps():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    px = py = pt = 0.0
    left = right = 0.0
    max_step = max_dtheta = 0.0
    for _ in range(2000):
        left += 0.04
        right += 0.05
        s = odom.update(left, right, 0.02)
        max_step = max(max_step, math.hypot(s.x - px, s.y - py))
        max_dtheta = max(max_dtheta, abs(normalize_angle(s.theta - pt)))
        px, py, pt = s.x, s.y, s.theta
    assert max_step < 0.01        # < 1 cm per update
    assert max_dtheta < 0.01      # < 0.01 rad per update


def test_heading_stays_normalized():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    left = right = 0.0
    for _ in range(3000):
        left -= 0.05
        right += 0.05
        s = odom.update(left, right, 0.02)
        assert -math.pi <= s.theta <= math.pi


def test_non_positive_dt_is_ignored_and_not_consumed():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    s = odom.update(1.0, 1.0, 0.0)               # same timestamp: ignored
    assert (s.x, s.y) == (0.0, 0.0)
    s = odom.update(1.0, 1.0, -0.5)              # out of order: ignored
    assert (s.x, s.y) == (0.0, 0.0)
    s = odom.update(1.0, 1.0, 0.1)               # delta still counted once
    assert s.x == pytest.approx(R * 1.0, rel=1e-9)


def test_quaternion():
    qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2)
    assert qx == 0.0 and qy == 0.0
    assert qz == pytest.approx(math.sin(math.pi / 4))
    assert qw == pytest.approx(math.cos(math.pi / 4))
    assert math.atan2(2 * qw * qz, 1 - 2 * qz * qz) == pytest.approx(math.pi / 2)


def test_velocity_smoothing_reduces_jitter_but_not_pose():
    raw, smooth = make(1.0), make(0.3)
    raw.update(0.0, 0.0, 0.0)
    smooth.update(0.0, 0.0, 0.0)
    left = right = 0.0
    raw_w, smooth_w = [], []
    for i in range(300):
        step = 0.05 if i % 3 else 0.10           # uneven arrival of wheel motion
        left -= step
        right += step
        raw_w.append(raw.update(left, right, 0.02).omega)
        smooth_w.append(smooth.update(left, right, 0.02).omega)
    assert (max(smooth_w[10:]) - min(smooth_w[10:])) < (max(raw_w[10:]) - min(raw_w[10:]))
    assert raw.state.theta == pytest.approx(smooth.state.theta, abs=1e-12)


def test_invalid_parameters():
    with pytest.raises(ValueError):
        WheelOdometry(0.0, L)
    with pytest.raises(ValueError):
        WheelOdometry(R, 0.0)
    with pytest.raises(ValueError):
        WheelOdometry(R, L, 0.0)

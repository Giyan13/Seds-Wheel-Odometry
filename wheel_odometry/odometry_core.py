"""Pure-Python differential-drive odometry math (no ROS imports).

Works in wheel angles (radians), exactly like /joint_states reports them.
Kept separate from the ROS node so it can be unit tested anywhere.
"""
import math
from dataclasses import dataclass


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float):
    """Return (x, y, z, w) for a rotation of `yaw` radians about Z (roll = pitch = 0)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class OdomState:
    x: float = 0.0       # metres
    y: float = 0.0       # metres
    theta: float = 0.0   # radians, in [-pi, pi]
    v: float = 0.0       # forward speed, m/s
    omega: float = 0.0   # turn rate, rad/s


class WheelOdometry:
    """Integrates left/right wheel positions (radians) into a 2D pose."""

    def __init__(self, wheel_radius: float, wheel_separation: float,
                 velocity_alpha: float = 1.0):
        """velocity_alpha in (0, 1]: optional smoothing of v and omega (1.0 = off).

        Pose is always integrated from raw wheel deltas; only the reported
        velocities are smoothed.
        """
        if wheel_radius <= 0 or wheel_separation <= 0:
            raise ValueError('wheel_radius and wheel_separation must be > 0')
        if not 0.0 < velocity_alpha <= 1.0:
            raise ValueError('velocity_alpha must be in (0, 1]')
        self.r = wheel_radius
        self.L = wheel_separation
        self.velocity_alpha = velocity_alpha
        self.reset()

    def reset(self):
        self.state = OdomState()
        self._prev_left = None
        self._prev_right = None
        self._have_velocity = False

    @property
    def has_baseline(self) -> bool:
        return self._prev_left is not None

    def update(self, left_pos: float, right_pos: float, dt: float) -> OdomState:
        """Feed wheel positions (rad) and seconds since the previous update."""
        # First valid sample: record the baseline only. Pose and velocity stay zero.
        if not self.has_baseline:
            self._prev_left = left_pos
            self._prev_right = right_pos
            return self.state

        # Out-of-order or duplicate timestamp: ignore, and do not consume the delta.
        if dt <= 0.0:
            return self.state

        # d = r * delta_phi  (radians -> ground distance)
        d_left = self.r * (left_pos - self._prev_left)
        d_right = self.r * (right_pos - self._prev_right)
        self._prev_left = left_pos
        self._prev_right = right_pos

        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.L

        # Midpoint heading, not the old heading.
        mid_theta = self.state.theta + d_theta / 2.0
        s = self.state
        s.x += d_center * math.cos(mid_theta)
        s.y += d_center * math.sin(mid_theta)
        s.theta = normalize_angle(s.theta + d_theta)

        raw_v = d_center / dt
        raw_omega = d_theta / dt
        if not self._have_velocity:
            s.v, s.omega = raw_v, raw_omega
            self._have_velocity = True
        else:
            a = self.velocity_alpha
            s.v += a * (raw_v - s.v)
            s.omega += a * (raw_omega - s.omega)
        return s

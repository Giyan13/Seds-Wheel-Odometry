"""Demo publisher: simulates encoder ticks (straight, then in-place turn, repeat).

Lets you test /odom (and Anosha's dashboard) without a real rover.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class FakeEncoders(Node):
    def __init__(self):
        super().__init__('fake_encoders')
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_base', 0.35)
        self.declare_parameter('ticks_per_rev', 1440.0)
        self.declare_parameter('speed', 0.3)       # m/s during straight phase
        self.declare_parameter('turn_rate', 0.5)   # rad/s during turn phase
        self.declare_parameter('phase_seconds', 5.0)

        p = self.get_parameter
        self.mpt = 2.0 * math.pi * p('wheel_radius').value / p('ticks_per_rev').value
        self.base = p('wheel_base').value
        self.speed = p('speed').value
        self.turn_rate = p('turn_rate').value
        self.phase = p('phase_seconds').value

        self.left = 0.0
        self.right = 0.0
        self.t = 0.0
        self.dt = 0.02  # 50 Hz
        self.pub_l = self.create_publisher(Int32, 'left_ticks', 10)
        self.pub_r = self.create_publisher(Int32, 'right_ticks', 10)
        self.create_timer(self.dt, self._tick)

    def _tick(self):
        straight = int(self.t / self.phase) % 2 == 0
        v = self.speed if straight else 0.0
        w = 0.0 if straight else self.turn_rate
        self.left += (v - w * self.base / 2.0) * self.dt / self.mpt
        self.right += (v + w * self.base / 2.0) * self.dt / self.mpt
        self.t += self.dt
        self.pub_l.publish(Int32(data=int(round(self.left))))
        self.pub_r.publish(Int32(data=int(round(self.right))))


def main(args=None):
    rclpy.init(args=args)
    node = FakeEncoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

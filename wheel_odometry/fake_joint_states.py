"""Demo publisher: fake /joint_states (straight 5 s, in-place turn 5 s, repeat).

Lets you test /wheel/odometry (and a dashboard) without the simulator.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointStates(Node):
    def __init__(self):
        super().__init__('fake_joint_states')
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('speed', 0.3)        # m/s in straight phase
        self.declare_parameter('turn_rate', 0.5)    # rad/s in turn phase
        self.declare_parameter('phase_seconds', 5.0)
        self.declare_parameter('rate', 50.0)        # Hz

        p = self.get_parameter
        self.r = p('wheel_radius').value
        self.L = p('wheel_separation').value
        self.names = [p('left_joint_name').value, p('right_joint_name').value]
        self.speed = p('speed').value
        self.turn_rate = p('turn_rate').value
        self.phase = p('phase_seconds').value
        self.dt = 1.0 / p('rate').value

        self.left = 0.0
        self.right = 0.0
        self.t = 0.0
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(self.dt, self._tick)

    def _tick(self):
        straight = int(self.t / self.phase) % 2 == 0
        v = self.speed if straight else 0.0
        w = 0.0 if straight else self.turn_rate
        # wheel angular speed = ground speed / radius
        self.left += (v - w * self.L / 2.0) / self.r * self.dt
        self.right += (v + w * self.L / 2.0) / self.r * self.dt
        self.t += self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [self.left, self.right]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStates()
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

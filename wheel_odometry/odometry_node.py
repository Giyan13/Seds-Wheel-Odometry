"""ROS 2 node: /joint_states (wheel angles) -> nav_msgs/Odometry on /wheel/odometry.

Follows robojackets-urc-software-training, Lesson 5.
Publishes the odometry MESSAGE only. It does NOT publish TF (the EKF owns odom -> base_footprint).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from wheel_odometry.odometry_core import WheelOdometry, yaw_to_quaternion


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.075)       # m, matches the URDF
        self.declare_parameter('wheel_separation', 0.34)    # m, matches the URDF
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('velocity_smoothing', 1.0)   # 1.0 = off (lesson default)

        p = self.get_parameter
        self.left_name = p('left_joint_name').value
        self.right_name = p('right_joint_name').value
        self.odom = WheelOdometry(
            p('wheel_radius').value, p('wheel_separation').value,
            p('velocity_smoothing').value)

        self.last_t = None

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.get_logger().info(
            f'Listening on /joint_states for "{self.left_name}" and "{self.right_name}"')

    def _on_joint_state(self, msg: JointState):
        # Need both wheel joints in this message.
        try:
            li = msg.name.index(self.left_name)
            ri = msg.name.index(self.right_name)
        except ValueError:
            return
        if li >= len(msg.position) or ri >= len(msg.position):
            return

        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:  # source does not stamp its messages: fall back to node clock
            now = self.get_clock().now()
            t = now.nanoseconds * 1e-9
            stamp = now.to_msg()

        if self.last_t is None:
            dt = 0.0          # first message: establish baseline
        else:
            dt = t - self.last_t
            if dt <= 0.0:
                return        # duplicate / out-of-order sample: ignore
        self.last_t = t

        s = self.odom.update(msg.position[li], msg.position[ri], dt)
        self._publish(s, stamp)

    def _publish(self, s, stamp):
        qx, qy, qz, qw = yaw_to_quaternion(s.theta)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = s.x
        msg.pose.pose.position.y = s.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = s.v
        msg.twist.twist.angular.z = s.omega
        # Self-declared diagonal covariance: small for x/y, larger for yaw.
        for cov in (msg.pose.covariance, msg.twist.covariance):
            cov[0] = 1e-3
            cov[7] = 1e-3
            cov[35] = 1e-2
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
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

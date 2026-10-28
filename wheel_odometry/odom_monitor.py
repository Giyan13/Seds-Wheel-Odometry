"""Readable live monitor for /wheel/odometry (for demos and proof recordings).

Prints one line per update, at a limited rate, e.g.:
  STRAIGHT | x=  1.234 m  y=  0.000 m  heading=   0.0 deg | v= 0.300 m/s  w= 0.000 rad/s
"""
import math


def yaw_from_quaternion(z: float, w: float) -> float:
    """Yaw (rad) of a planar rotation stored as quaternion (0, 0, z, w)."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def classify_motion(v: float, omega: float, v_eps: float = 0.02, w_eps: float = 0.02) -> str:
    moving = abs(v) > v_eps
    turning = abs(omega) > w_eps
    if moving and turning:
        return 'ARC     '
    if moving:
        return 'STRAIGHT'
    if turning:
        return 'TURNING '
    return 'STOPPED '


def format_line(x, y, yaw, v, omega) -> str:
    return (f'{classify_motion(v, omega)} | x={x:8.3f} m  y={y:8.3f} m  '
            f'heading={math.degrees(yaw):7.1f} deg | v={v:6.3f} m/s  w={omega:6.3f} rad/s')


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    class OdomMonitor(Node):
        def __init__(self):
            super().__init__('odom_monitor')
            self.declare_parameter('print_rate', 5.0)  # lines per second
            self.min_period = 1.0 / self.get_parameter('print_rate').value
            self.last_print = -1e9
            self.create_subscription(Odometry, '/wheel/odometry', self._cb, 10)

        def _cb(self, msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_print < self.min_period:
                return
            self.last_print = now
            o = msg.pose.pose.orientation
            yaw = yaw_from_quaternion(o.z, o.w)
            print(format_line(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                              msg.twist.twist.linear.x, msg.twist.twist.angular.z),
                  flush=True)

    rclpy.init(args=args)
    node = OdomMonitor()
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

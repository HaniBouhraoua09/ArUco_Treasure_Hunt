#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool


class RotationControllerNode(Node):
    def __init__(self):
        super().__init__('rotation_controller_node')

        # Configurable ROS 2 parameters
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('k_yaw', 1.2)
        self.declare_parameter('yaw_tolerance_rad', 0.03)

        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.current_yaw = None
        self.target_yaw = 0.0
        self.rotating = False

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.done_pub = self.create_publisher(Bool, '/treasure_hunt/rotation_done', 10)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Float64, '/treasure_hunt/rotate_delta', self.rotate_cmd_callback, 10)

        self.get_logger().info(f'RotationControllerNode started. Listening on {odom_topic}')

    def rotate_cmd_callback(self, msg: Float64):
        """Receives desired relative rotation angle (in radians) from state_machine_node."""
        if self.current_yaw is None:
            self.get_logger().warn('Cannot start rotation: waiting for odometry message...')
            return

        delta_yaw = msg.data
        raw_target = self.current_yaw + delta_yaw
        # Normalize target_yaw in [-pi, pi]
        self.target_yaw = math.atan2(math.sin(raw_target), math.cos(raw_target))
        self.rotating = True
        self.get_logger().info(
            f'Starting closed-loop rotation: delta={math.degrees(delta_yaw):.1f} deg, '
            f'target_yaw={math.degrees(self.target_yaw):.1f} deg'
        )

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler yaw
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )

        if not self.rotating:
            return

        # Compute shortest angular error normalized in [-pi, pi]
        err = math.atan2(
            math.sin(self.target_yaw - self.current_yaw),
            math.cos(self.target_yaw - self.current_yaw)
        )

        yaw_tol = self.get_parameter('yaw_tolerance_rad').value
        k_yaw = self.get_parameter('k_yaw').value

        twist = Twist()
        if abs(err) > yaw_tol:
            twist.angular.z = max(min(k_yaw * err, 0.8), -0.8)
            self.cmd_pub.publish(twist)
        else:
            # Target heading reached -> stop robot and notify state machine
            self.cmd_pub.publish(Twist())
            self.rotating = False
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            self.get_logger().info(
                f'Rotation finished at yaw={math.degrees(self.current_yaw):.1f} deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RotationControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
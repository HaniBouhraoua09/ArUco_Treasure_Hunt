#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')

        # Configurable ROS 2 parameters
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('f_sim', 381.36)
        self.declare_parameter('target_px_sim', 166.0)
        self.declare_parameter('k_w', 0.003)
        self.declare_parameter('k_v', 0.003)
        self.declare_parameter('size_tolerance_px', 4.0)
        self.declare_parameter('center_tolerance_px', 15.0)

        camera_info_topic = self.get_parameter('camera_info_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Default camera intrinsics (updated automatically from /camera/camera_info)
        self.cx = 320.0
        self.s_d = self.get_parameter('target_px_sim').value
        self.calibrated = False

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.reached_pub = self.create_publisher(Int32, '/treasure_hunt/servo_reached', 10)

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/treasure_hunt/servo_target', self.servo_target_callback, 10
        )

        self.get_logger().info('VisualServoingNode started.')

    def camera_info_callback(self, msg: CameraInfo):
        """Reads fx and cx from the K matrix and scales s_d for Gazebo and ROSbot."""
        if not self.calibrated and len(msg.k) == 9 and msg.k[0] > 0.0:
            fx_current = msg.k[0]
            self.cx = msg.k[2]
            f_sim = self.get_parameter('f_sim').value
            target_sim = self.get_parameter('target_px_sim').value

            # Pinhole focal-length scaling: s_d = 166 * (f_ROSbot / f_sim)
            self.s_d = target_sim * (fx_current / f_sim)
            self.calibrated = True
            self.get_logger().info(
                f'Camera calibrated: fx={fx_current:.2f}, cx={self.cx:.1f} -> Scaled s_d={self.s_d:.1f} px'
            )

    def servo_target_callback(self, msg: Float64MultiArray):
        """
        Receives [active_flag, marker_id, u, v, s] from state_machine_node.
        Computes IBVS velocities or stops when within tolerance.
        """
        if len(msg.data) < 5 or msg.data[0] < 0.5:
            return

        marker_id = int(msg.data[1])
        u = msg.data[2]
        s = msg.data[4]

        k_w = self.get_parameter('k_w').value
        k_v = self.get_parameter('k_v').value
        size_tol = self.get_parameter('size_tolerance_px').value
        center_tol = self.get_parameter('center_tolerance_px').value

        # Feature errors in image space
        err_u = u - self.cx
        err_s = self.s_d - s

        twist = Twist()

        # Check if marker is centered and apparent size matches s_d within tolerance
        if abs(err_s) <= size_tol and abs(err_u) <= center_tol:
            self.cmd_pub.publish(Twist())  # Stop robot
            reached_msg = Int32()
            reached_msg.data = marker_id
            self.reached_pub.publish(reached_msg)
            self.get_logger().info(
                f'Marker {marker_id} reached! (s={s:.1f} px, target s_d={self.s_d:.1f} px)'
            )
        else:
            # Proportional IBVS control laws:
            # Align: w = -k_w * (u - c_x)
            # Approach: v = k_v * (s_d - s)
            w = -k_w * err_u
            v = k_v * err_s

            twist.angular.z = max(min(w, 0.6), -0.6)
            twist.linear.x = max(min(v, 0.25), -0.15)
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
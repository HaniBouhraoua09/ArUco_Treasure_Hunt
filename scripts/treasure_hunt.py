#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class ArUcoTreasureHunt(Node):
    def __init__(self):
        super().__init__('aruco_treasure_hunt')

        # Configurable ROS 2 Parameters (ROSbot 3 raw topics)
        self.declare_parameter('image_topic', '/rosbot_3/camera/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/rosbot_3/camera/rgb/camera_info')
        self.declare_parameter('odom_topic', '/rosbot_3/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/rosbot_3/cmd_vel')
        self.declare_parameter('marker_image_topic', '/marker_image')

        # Real-world tolerances & safe lab speeds
        self.declare_parameter('f_sim', 381.3624)      # Gazebo 640x480, HFOV=1.3962634
        self.declare_parameter('s_d_sim', 166.0)       # Target pixel size in sim
        self.declare_parameter('s_tolerance', 18.0)    # Widened stopping tolerance (px)
        self.declare_parameter('u_tolerance', 45.0)    # Widened horizontal center tolerance (px)
        self.declare_parameter('k_w', 0.004)           # IBVS angular gain
        self.declare_parameter('k_v', 0.004)           # IBVS linear gain
        self.declare_parameter('search_vel', 0.20)     # Safe forward search speed (m/s)
        self.declare_parameter('k_yaw', 1.5)           # Closed-loop rotation P-gain
        self.declare_parameter('yaw_tolerance', 0.04)  # Rotation stopping threshold (rad)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        marker_image_topic = self.get_parameter('marker_image_topic').value

        self.f_sim = float(self.get_parameter('f_sim').value)
        self.s_d_sim = float(self.get_parameter('s_d_sim').value)
        self.s_tol = float(self.get_parameter('s_tolerance').value)
        self.u_tol = float(self.get_parameter('u_tolerance').value)
        self.k_w = float(self.get_parameter('k_w').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.search_vel = float(self.get_parameter('search_vel').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)

        # Dynamic camera parameters (updated by /camera_info)
        self.fx = self.f_sim
        self.cx = 320.0
        self.s_d = self.s_d_sim
        self.camera_info_received = False

        # State Machine: 'SEARCH_AND_SERVO', 'ROTATING', 'DONE'
        self.state = 'SEARCH_AND_SERVO'
        self.visited_ids = set()
        self.current_yaw = 0.0
        self.odom_received = False
        self.target_yaw = 0.0

        # ArUco setup (DICT_ARUCO_ORIGINAL)
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            if hasattr(cv2.aruco, 'DetectorParameters_create'):
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            else:
                self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = None

        self.bridge = CvBridge()

        # Publishers & SensorData QoS Subscribers (prevents Wi-Fi queue buildup & clean Ctrl+C)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_img_pub = self.create_publisher(Image, marker_image_topic, 10)

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)

        # 20 Hz control timer for closed-loop rotations
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('ArUco Treasure Hunt node started. Searching for Marker 1...')

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def camera_info_cb(self, msg: CameraInfo):
        if not self.camera_info_received and len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            # Scale desired pixel size s_d for ROSbot vs Simulation camera
            self.s_d = self.s_d_sim * (self.fx / self.f_sim)
            self.camera_info_received = True
            self.get_logger().info(
                f'CameraInfo received: fx={self.fx:.1f}, cx={self.cx:.1f} -> Scaled s_d={self.s_d:.1f} px'
            )

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Quaternion to Euler yaw
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )
        self.odom_received = True

    def detect_aruco(self, img):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(img)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                img, self.aruco_dict, parameters=self.aruco_params
            )
        return corners, ids

    def image_cb(self, msg: Image):
        if self.state != 'SEARCH_AND_SERVO' or not self.odom_received:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        corners, ids = self.detect_aruco(img)
        twist = Twist()

        # Filter for unvisited valid markers (1, 2, 3, 4)
        valid_now = []
        if ids is not None:
            for i, m_id_arr in enumerate(ids):
                m_id = int(m_id_arr[0])
                if m_id not in self.visited_ids and m_id in (1, 2, 3, 4):
                    pts = corners[i][0]  # shape (4, 2)
                    side_lengths = [
                        np.linalg.norm(pts[(j + 1) % 4] - pts[j]) for j in range(4)
                    ]
                    s = float(np.mean(side_lengths))
                    u = float(np.mean(pts[:, 0]))
                    v_px = float(np.mean(pts[:, 1]))
                    valid_now.append((s, m_id, u, v_px))

        if not valid_now:
            # No unvisited marker in view: drive straight ahead
            twist.linear.x = self.search_vel
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Track the closest unvisited marker (largest apparent size s)
        valid_now.sort(key=lambda x: x[0], reverse=True)
        s, marker_id, u, v_px = valid_now[0]

        err_s = self.s_d - s
        err_u = u - self.cx

        # Trigger stop as soon as marker reaches stopping size and is centered
        if s >= (self.s_d - self.s_tol) and abs(err_u) <= self.u_tol:
            # Stop the robot
            self.cmd_pub.publish(Twist())
            self.visited_ids.add(marker_id)

            # Requirement 1: Draw circle and ID, then publish on /marker_image
            cx_int, cy_int = int(round(u)), int(round(v_px))
            r_int = int(round(s / math.sqrt(2.0)))
            annotated = img.copy()
            cv2.circle(annotated, (cx_int, cy_int), r_int, (0, 255, 0), 3)
            cv2.putText(
                annotated, f"id {marker_id}", (cx_int - r_int, cy_int - r_int - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.marker_img_pub.publish(out_msg)

            self.handle_marker_instruction(marker_id)
        else:
            # Requirement 2: Image-Based Visual Servoing (IBVS)
            w = -self.k_w * err_u
            v = self.k_v * err_s

            # Prevent motor stall right before reaching stopping distance
            if abs(err_s) > self.s_tol and abs(v) < 0.11:
                v = math.copysign(0.11, v)

            twist.linear.x = max(-0.25, min(0.25, v))
            twist.angular.z = max(-0.65, min(0.65, w))
            self.cmd_pub.publish(twist)

    def handle_marker_instruction(self, marker_id: int):
        if marker_id == 1:
            # Turn right by 90 degrees (-pi/2)
            self.target_yaw = self.normalize_angle(self.current_yaw - math.pi / 2.0)
            self.state = 'ROTATING'
            self.get_logger().info(f'Reached Marker 1! Turning right 90 deg to yaw={self.target_yaw:.2f} rad')
        elif marker_id == 2:
            # Rotate by 180 degrees (+pi)
            self.target_yaw = self.normalize_angle(self.current_yaw + math.pi)
            self.state = 'ROTATING'
            self.get_logger().info(f'Reached Marker 2! Rotating 180 deg to yaw={self.target_yaw:.2f} rad')
        elif marker_id == 3:
            # Turn left by 90 degrees (+pi/2)
            self.target_yaw = self.normalize_angle(self.current_yaw + math.pi / 2.0)
            self.state = 'ROTATING'
            self.get_logger().info(f'Reached Marker 3! Turning left 90 deg to yaw={self.target_yaw:.2f} rad')
        elif marker_id == 4:
            # Treasure found! Stop completely.
            self.state = 'DONE'
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Found! Stop: the hunt is over.')

    def control_loop(self):
        if self.state == 'ROTATING' and self.odom_received:
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            twist = Twist()
            if abs(err) <= self.yaw_tol:
                # Rotation finished -> resume forward search
                self.cmd_pub.publish(Twist())
                self.state = 'SEARCH_AND_SERVO'
                self.get_logger().info('Rotation complete! Going ahead to search for next marker...')
            else:
                w = self.k_yaw * err
                # Minimum 0.50 rad/s to overcome 4WD skid-steer tire friction
                if abs(w) < 0.50:
                    w = math.copysign(0.50, w)
                twist.angular.z = max(-0.75, min(0.75, w))
                self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoTreasureHunt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
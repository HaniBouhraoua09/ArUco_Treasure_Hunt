#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int32


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Configurable ROS 2 parameters
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('marker_image_topic', '/marker_image')
        self.declare_parameter('marker_size_m', 0.2)

        image_topic = self.get_parameter('image_topic').value
        marker_image_topic = self.get_parameter('marker_image_topic').value

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header = None
        self.latest_markers = {}  # marker_id -> (cx, cy, s)

        # Publishers
        self.detections_pub = self.create_publisher(
            Float64MultiArray, '/treasure_hunt/detections', 10
        )
        self.marker_img_pub = self.create_publisher(
            Image, marker_image_topic, 10
        )

        # Subscribers
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(
            Int32, '/treasure_hunt/publish_annotated_id', self.publish_annotated_callback, 10
        )

        # Configure ArUco detector with DICT_ARUCO_ORIGINAL
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

        self.get_logger().info(f'PerceptionNode started. Listening on {image_topic}')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        self.latest_frame = frame
        self.latest_header = msg.header
        self.latest_markers.clear()

        # Detect markers in the image
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame, self.aruco_dict, parameters=self.aruco_params
            )

        flat_detections = []

        if ids is not None:
            for i, marker_id_arr in enumerate(ids):
                marker_id = int(marker_id_arr[0])
                pts = corners[i][0]  # 4 corners: (x0,y0), (x1,y1), (x2,y2), (x3,y3)

                # Compute center pixel coordinates (u, v)
                u = float(np.mean(pts[:, 0]))
                v = float(np.mean(pts[:, 1]))

                # Compute apparent side length s in pixels (average of the 4 edges)
                side_lengths = [
                    np.linalg.norm(pts[0] - pts[1]),
                    np.linalg.norm(pts[1] - pts[2]),
                    np.linalg.norm(pts[2] - pts[3]),
                    np.linalg.norm(pts[3] - pts[0])
                ]
                s = float(np.mean(side_lengths))

                self.latest_markers[marker_id] = (u, v, s)
                flat_detections.extend([float(marker_id), u, v, s])

        # Publish all detected markers in current frame: [id, u, v, s, ...]
        det_msg = Float64MultiArray()
        det_msg.data = flat_detections
        self.detections_pub.publish(det_msg)

    def publish_annotated_callback(self, msg: Int32):
        """Triggered when the robot reaches a marker to publish on /marker_image."""
        marker_id = msg.data
        if self.latest_frame is None or marker_id not in self.latest_markers:
            return

        img = self.latest_frame.copy()
        u, v, s = self.latest_markers[marker_id]
        cx, cy = int(u), int(v)
        r = int(s / math.sqrt(2.0))

        # Draw circle and marker ID 
        cv2.circle(img, (cx, cy), r, (0, 255, 0), 3)
        cv2.putText(
            img, f"id {marker_id}", (cx - r, cy - r - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
        )

        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg.header = self.latest_header
        self.marker_img_pub.publish(out_msg)
        self.get_logger().info(f'Published annotated image for Marker {marker_id} on /marker_image')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import math
import time
from enum import Enum

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from aruco_navigator.marker_tracking import CornerTracker, detect_with_fallback


class State(Enum):
    SEARCHING = 1
    ALIGNING = 2
    APPROACHING = 3
    EXECUTING = 4
    SUCCESS = 5
    RECOVERING = 6


class ArucoNavigatorNode(Node):
    def __init__(self):
        super().__init__("aruco_navigator")
        defaults = {
            "image_topic": "/camera/image",
            "odom_topic": "/odom",
            "cmd_vel_topic": "/cmd_vel",
            "camera_info_topic": "/camera/camera_info",
            "marker_image_topic": "/marker_image",
            "target_marker_size_px": 166.0,
            "marker_image_raw_topic": "/marker_image_raw",
            "marker_size_m": 0.2,
            "sim_focal_length": 381.36,
            "kp_yaw": 0.003,
            "kp_dist": 0.0015,
            "max_linear_vel": 0.18,
            "max_angular_vel": 0.35,
            "min_linear_vel": 0.035,
            "min_angular_vel": 0.06,
            "dist_tolerance_px": 6.0,
            "yaw_tolerance_px": 12.0,
            "rotation_tolerance_rad": 0.04,
            "search_angular_vel": 0.12,
            "max_align_angular_vel": 0.35,
            "search_linear_vel": 0.10,
            "search_heading_kp": 0.8,
            # Acquire distant markers before forward transit carries us past them.
            "min_detected_marker_side_px": 12.0,
            "marker_confirm_frames": 5,
            "alignment_forward_speed": 0.08,
            "alignment_stop_error_fraction": 0.15,
            "alignment_near_target_ratio": 0.8,
            "marker_timeout_sec": 4.0,
            "control_period_sec": 0.1,
            "observation_stale_sec": 0.15,
            "tracking_max_duration_sec": 1.0,
            "sensor_timeout_sec": 0.5,
            "image_max_latency_sec": 0.5,
            "recovery_speed": 0.06,
            "recovery_max_distance_m": 0.4,
            "recovery_timeout_sec": 8.0,
            "recovery_far_target_ratio": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),
            cv2.aruco.DetectorParameters(),
        )
        fallback_parameters = cv2.aruco.DetectorParameters()
        fallback_parameters.adaptiveThreshWinSizeMin = 5
        fallback_parameters.adaptiveThreshWinSizeMax = 35
        fallback_parameters.adaptiveThreshWinSizeStep = 2
        self.fallback_detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),
            fallback_parameters,
        )
        self.tracker = CornerTracker(float(self.p["tracking_max_duration_sec"]))
        self.last_image_wall = self.last_odom_wall = None
        self.calibrated = False
        self.observation_source = None
        self.observation_image = self.observation_header = None
        self.position = None
        self.observation_bearing = None
        self.recovery_start = self.recovery_started = None
        self.state = State.SEARCHING
        # Assignment starts at the origin facing +x, toward marker 1.
        self.transit_forward = True
        self.used = set()
        self.marker_id = None
        self.candidate_id = None
        self.candidate_count = 0
        self.observation = None
        self.last_seen = None
        self.yaw = None
        self.target_yaw = None
        self.fx_ratio = 1.0
        self.bridge_img_pub = self.create_publisher(
            Image, self.p["marker_image_raw_topic"], 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.p["cmd_vel_topic"], 10)
        self.create_subscription(
            Image, self.p["image_topic"], self.on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.p["odom_topic"], self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            self.p["camera_info_topic"],
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(float(self.p["control_period_sec"]), self.control)
        self.get_logger().info("ArUco treasure hunt ready (DICT_ARUCO_ORIGINAL).")

    def on_camera_info(self, msg):
        fx = float(msg.k[0])
        sim_fx = float(self.p["sim_focal_length"])
        if math.isfinite(fx) and fx > 0 and sim_fx > 0:
            self.fx_ratio = fx / sim_fx
            self.calibrated = True

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        position = msg.pose.pose.position
        self.position = (position.x, position.y)
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        self.last_odom_wall = time.monotonic() if math.isfinite(self.yaw) else None
        if self.target_yaw is None:
            self.target_yaw = self.yaw

    def on_image(self, msg):
        now = self.get_clock().now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = now.nanoseconds / 1e9 - stamp
        if age < -0.05 or age > float(self.p["image_max_latency_sec"]):
            return
        self.latest_header = msg.header
        try:
            if msg.encoding == "bgr8":
                image = (
                    np.frombuffer(msg.data, dtype=np.uint8)
                    .reshape(msg.height, msg.step)[:, : msg.width * 3]
                    .reshape(msg.height, msg.width, 3)
                    .copy()
                )
            elif msg.encoding == "rgb8":
                image = cv2.cvtColor(
                    np.frombuffer(msg.data, dtype=np.uint8)
                    .reshape(msg.height, msg.step)[:, : msg.width * 3]
                    .reshape(msg.height, msg.width, 3),
                    cv2.COLOR_RGB2BGR,
                )
            else:
                raise ValueError(f"Unsupported image encoding: {msg.encoding}")
        except Exception as exc:
            self.get_logger().warning(f"Could not decode image: {exc}")
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.last_image_wall = time.monotonic()
        if self.state in (State.EXECUTING, State.SUCCESS):
            self.tracker.clear()
            self.latest_image = image
            return
        corners, ids = detect_with_fallback(
            gray, self.detector, self.fallback_detector, self.expected_marker_id()
        )
        visible = []
        if ids is not None:
            for corner, raw_id in zip(corners, ids.flatten()):
                mid = int(raw_id)
                pts = corner.reshape(4, 2)
                center = pts.mean(axis=0)
                side = float(
                    np.mean(
                        [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
                    )
                )

                if mid != self.expected_marker_id():
                    continue

                if mid != self.marker_id and side < float(
                    self.p["min_detected_marker_side_px"]
                ):
                    continue
                visible.append((mid, pts, center, side))
        # While searching, confirm even a returning target before resuming motion.
        chosen = None
        if self.state in (State.ALIGNING, State.APPROACHING):
            chosen = next((item for item in visible if item[0] == self.marker_id), None)
        if (
            chosen is None
            and visible
            and self.state in (State.SEARCHING, State.RECOVERING)
        ):
            candidate = max(visible, key=lambda item: item[3])
            if candidate[0] == self.candidate_id:
                self.candidate_count += 1
            else:
                self.candidate_id, self.candidate_count = candidate[0], 1
            if self.candidate_count >= int(self.p["marker_confirm_frames"]):
                chosen = candidate
                self.marker_id = chosen[0]
                self.state = State.ALIGNING
                self.transit_forward = False
                self._alignment_logged = False
                self.get_logger().info(
                    f"Marker {self.marker_id} acquired stably: side={chosen[3]:.1f}px, "
                    f"u={chosen[2][0]:.1f}px."
                )
        elif self.state in (State.SEARCHING, State.RECOVERING):
            self.candidate_id, self.candidate_count = None, 0
        if chosen is not None:
            mid, pts, center, side = chosen
            self.tracker.seed(gray, pts, mid, stamp)
            self.observation_source = "decoded"
        elif self.state in (State.ALIGNING, State.APPROACHING):
            pts = self.tracker.update(gray, self.marker_id, stamp)
            if pts is not None:
                center = pts.mean(axis=0)
                side = float(
                    np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1).mean()
                )
                chosen = (self.marker_id, pts, center, side)
                self.observation_source = "tracked"
        if chosen is not None:
            mid, pts, center, side = chosen
            self.observation = (float(center[0]), side)
            if self.yaw is not None:
                self.observation_bearing = self.normalize(
                    self.yaw
                    - math.atan2(
                        float(center[0]) - image.shape[1] / 2,
                        float(self.p["sim_focal_length"]) * self.fx_ratio,
                    )
                )
            self.last_seen = now
            cx, cy = int(center[0]), int(center[1])
            radius = max(4, int(np.mean(np.linalg.norm(pts - center, axis=1))))
            cv2.circle(image, (cx, cy), radius, (0, 255, 0), 3)
            cv2.putText(
                image,
                f"id {mid}",
                (cx - radius, cy - radius - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            self.observation_image, self.observation_header = image.copy(), msg.header
        self.latest_image = image

    @staticmethod
    def normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def expected_marker_id(self):
        return next((mid for mid in (1, 2, 3, 4) if mid not in self.used), None)

    def stop(self):
        try:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception as exc:
            self.get_logger().debug(
                f"Final zero velocity was not publishable during shutdown: {exc}"
            )

    def control(self):
        cmd = Twist()
        if self.state == State.SUCCESS:
            self.cmd_pub.publish(cmd)
            return
        wall = time.monotonic()
        if (
            not self.calibrated
            or self.last_image_wall is None
            or self.last_odom_wall is None
            or wall - self.last_image_wall > float(self.p["sensor_timeout_sec"])
            or wall - self.last_odom_wall > float(self.p["sensor_timeout_sec"])
        ):
            self.cmd_pub.publish(cmd)
            return
        if self.state == State.RECOVERING:
            distance = math.dist(self.position, self.recovery_start)
            elapsed = (self.get_clock().now() - self.recovery_started).nanoseconds / 1e9
            if distance < float(self.p["recovery_max_distance_m"]) and elapsed < float(
                self.p["recovery_timeout_sec"]
            ):
                error = self.normalize(self.observation_bearing - self.yaw)
                cmd.angular.z = float(np.clip(0.8 * error, -0.2, 0.2))
                if abs(error) < 0.2:
                    cmd.linear.x = min(
                        float(self.p["recovery_speed"]), float(self.p["max_linear_vel"])
                    )
            # Once the bounded advance is exhausted, hold while still detecting.
        elif self.state == State.SEARCHING:
            if self.transit_forward:
                # Advance along the completed turn's heading to find the next box.
                cmd.linear.x = float(self.p["search_linear_vel"])
                if self.yaw is not None and self.target_yaw is not None:
                    heading_error = self.normalize(self.target_yaw - self.yaw)
                    cmd.angular.z = float(
                        np.clip(
                            float(self.p["search_heading_kp"]) * heading_error,
                            -float(self.p["max_angular_vel"]),
                            float(self.p["max_angular_vel"]),
                        )
                    )
            else:
                # With no active marker, rotate in place. Never drive forward blindly.
                cmd.angular.z = float(self.p["search_angular_vel"])
        elif self.state in (State.ALIGNING, State.APPROACHING):
            age = (
                None
                if self.last_seen is None
                else (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
            )
            target = (
                float(self.p["target_marker_size_px"])
                * self.fx_ratio
                * float(self.p["marker_size_m"])
                / 0.2
            )
            if (
                age is not None
                and age > float(self.p["observation_stale_sec"])
                and self.observation is not None
                and self.position is not None
                and self.observation_bearing is not None
                and self.observation[1]
                < target * float(self.p["recovery_far_target_ratio"])
            ):
                self.state = State.RECOVERING
                self.recovery_start = self.position
                self.recovery_started = self.get_clock().now()
                self.candidate_id, self.candidate_count = None, 0
                self.tracker.clear()
                self.get_logger().warning(
                    "Distant marker lost; advancing slowly toward last visual bearing, with distance/time limits."
                )
            elif (
                self.observation is None
                or age is None
                or age > float(self.p["marker_timeout_sec"])
            ):
                self.state, self.observation = State.SEARCHING, None
                self.transit_forward = False
                self.candidate_id, self.candidate_count = None, 0
                self.get_logger().warning(
                    "Marker lost; stopped forward travel and reacquiring in place."
                )
                if self.yaw is not None:
                    self.target_yaw = self.yaw
            elif age > float(self.p["observation_stale_sec"]):
                # Do not continue steering or driving from an outdated pixel error.
                pass
            else:
                u, side = self.observation
                err_x = u - self.latest_image.shape[1] / 2
                if self.state == State.ALIGNING and abs(err_x) <= float(
                    self.p["yaw_tolerance_px"]
                ):
                    self.state = State.APPROACHING
                if self.state == State.ALIGNING and not getattr(
                    self, "_alignment_logged", False
                ):
                    self.get_logger().info(
                        f"Aligning marker {self.marker_id}: u-cx={err_x:.1f}px, side={side:.1f}px."
                    )
                    self._alignment_logged = True
                if (
                    self.state == State.APPROACHING
                    and abs(err_x) > float(self.p["yaw_tolerance_px"]) * 1.5
                ):
                    self.state = State.ALIGNING
                cmd.angular.z = float(
                    np.clip(
                        -float(self.p["kp_yaw"]) * err_x,
                        -float(self.p["max_align_angular_vel"]),
                        float(self.p["max_align_angular_vel"]),
                    )
                )
                if abs(cmd.angular.z) < float(self.p["min_angular_vel"]) and abs(
                    err_x
                ) > float(self.p["yaw_tolerance_px"]):
                    cmd.angular.z = math.copysign(
                        float(self.p["min_angular_vel"]), cmd.angular.z
                    )
                target = (
                    float(self.p["target_marker_size_px"])
                    * self.fx_ratio
                    * float(self.p["marker_size_m"])
                    / 0.2
                )
                err_size = target - side
                if (
                    self.state == State.APPROACHING
                    and abs(err_x) <= float(self.p["yaw_tolerance_px"])
                    and self.observation_source == "decoded"
                    and abs(err_size) <= float(self.p["dist_tolerance_px"])
                ):
                    self.reached()
                    cmd = Twist()
                elif self.state == State.APPROACHING:
                    cmd.linear.x = float(
                        np.clip(
                            float(self.p["kp_dist"]) * err_size,
                            -float(self.p["max_linear_vel"]),
                            float(self.p["max_linear_vel"]),
                        )
                    )
                    if cmd.linear.x > 0 and cmd.linear.x < float(
                        self.p["min_linear_vel"]
                    ):
                        cmd.linear.x = float(self.p["min_linear_vel"])
                elif self.state == State.ALIGNING and side < target * float(
                    self.p["alignment_near_target_ratio"]
                ):
                    # Correct heading while travelling, slowing to zero as the
                    # target moves away from the center. Near the box, align first.
                    centered = float(self.p["yaw_tolerance_px"])
                    stop_error = max(
                        centered + 1.0,
                        self.latest_image.shape[1]
                        * float(self.p["alignment_stop_error_fraction"]),
                    )
                    alignment_scale = float(
                        np.clip(
                            (stop_error - abs(err_x)) / (stop_error - centered),
                            0.0,
                            1.0,
                        )
                    )
                    forward = max(
                        0.0,
                        min(
                            float(self.p["alignment_forward_speed"]),
                            float(self.p["max_linear_vel"]),
                            float(self.p["kp_dist"]) * err_size,
                        ),
                    )
                    cmd.linear.x = forward * alignment_scale
        elif self.state == State.EXECUTING:
            if self.yaw is not None:
                error = self.normalize(self.target_yaw - self.yaw)
                if abs(error) <= float(self.p["rotation_tolerance_rad"]):
                    self.get_logger().info(
                        f"Marker {self.marker_id} instruction complete."
                    )
                    if self.marker_id == 4:
                        self.state = State.SUCCESS
                        self.get_logger().info("TREASURE FOUND! Hunt complete.")
                    else:
                        self.state, self.observation, self.marker_id = (
                            State.SEARCHING,
                            None,
                            None,
                        )
                        self.transit_forward = True
                else:
                    cmd.angular.z = float(
                        np.clip(
                            0.8 * error,
                            -float(self.p["max_angular_vel"]),
                            float(self.p["max_angular_vel"]),
                        )
                    )
                    if abs(cmd.angular.z) < float(self.p["min_angular_vel"]):
                        cmd.angular.z = math.copysign(
                            float(self.p["min_angular_vel"]), error
                        )
        self.cmd_pub.publish(cmd)

    def reached(self):
        if self.marker_id == 4:
            self.publish_reached_image()
            self.used.add(4)
            self.state = State.SUCCESS
            self.get_logger().info("TREASURE FOUND! Hunt complete.")
        else:
            # Mark it consumed immediately so it cannot be reacquired during the turn.
            self.used.add(self.marker_id)
            self._alignment_logged = False
            self.transit_forward = False
            self.publish_reached_image()
            if self.yaw is None:
                self.get_logger().warning("No odometry received; holding position.")
                return
            turns = {1: -math.pi / 2, 2: math.pi, 3: math.pi / 2}
            self.target_yaw = self.normalize(self.yaw + turns[self.marker_id])
            self.state = State.EXECUTING
            self.get_logger().info(
                f"Reached marker {self.marker_id}; turning to yaw {self.target_yaw:.3f}."
            )

    def publish_reached_image(self):
        if self.observation_image is None:
            return
        out = Image()
        out.header = self.observation_header
        out.height, out.width = self.observation_image.shape[:2]
        out.encoding, out.is_bigendian = "bgr8", 0
        out.step = out.width * 3
        out.data = np.ascontiguousarray(self.observation_image).tobytes()
        self.bridge_img_pub.publish(out)

    def publish_status(self):
        import json

        msg = String()
        msg.data = json.dumps(
            {
                "state": self.state.name,
                "marker_id": self.marker_id,
                "used": sorted(self.used),
                "observation": self.observation,
                "source": self.observation_source,
                "observation_age": (
                    None
                    if self.last_seen is None
                    else (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
                ),
                "calibrated": self.calibrated,
            }
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

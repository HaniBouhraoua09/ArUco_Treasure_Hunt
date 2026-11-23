from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Defaults are for the real ROSbot; the simulation passes its own topics.
    robot = LaunchConfiguration("robot")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="rosbot_1",
                description="ROSbot namespace: rosbot_1, rosbot_2 or rosbot_3",
            ),
            DeclareLaunchArgument(
                "image_topic", default_value=["/", robot, "/camera/rgb/image_raw"]
            ),
            DeclareLaunchArgument(
                "image_transport",
                default_value="raw",
                description="raw, or compressed to subscribe to image_topic/compressed",
            ),
            DeclareLaunchArgument(
                "odom_topic", default_value=["/", robot, "/odometry/filtered"]
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value=["/", robot, "/cmd_vel"]),
            DeclareLaunchArgument(
                "camera_info_topic", default_value=["/", robot, "/camera/rgb/camera_info"]
            ),
            DeclareLaunchArgument("marker_image_topic", default_value="/marker_image"),
            DeclareLaunchArgument(
                "marker_image_raw_topic", default_value="/marker_image_raw"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("min_detected_marker_side_px", default_value="12.0"),
            DeclareLaunchArgument("alignment_forward_speed", default_value="0.08"),
            DeclareLaunchArgument(
                "alignment_stop_error_fraction", default_value="0.15"
            ),
            DeclareLaunchArgument("alignment_near_target_ratio", default_value="0.8"),
            DeclareLaunchArgument("max_linear_accel", default_value="0.3"),
            DeclareLaunchArgument("max_angular_accel", default_value="1.0"),
            Node(
                package="erl1",
                executable="marker_image_transport",
                name="marker_image_transport",
                output="screen",
                parameters=[
                    {
                        "raw_topic": LaunchConfiguration("marker_image_raw_topic"),
                        "output_topic": LaunchConfiguration("marker_image_topic"),
                    }
                ],
            ),
            Node(
                package="aruco_navigator",
                executable="aruco_navigator_node",
                name="aruco_navigator",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "image_transport": LaunchConfiguration("image_transport"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "marker_image_topic": LaunchConfiguration("marker_image_topic"),
                        "marker_image_raw_topic": LaunchConfiguration(
                            "marker_image_raw_topic"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        **{
                            name: ParameterValue(
                                LaunchConfiguration(name), value_type=float
                            )
                            for name in (
                                "min_detected_marker_side_px",
                                "alignment_forward_speed",
                                "alignment_stop_error_fraction",
                                "alignment_near_target_ratio",
                                "max_linear_accel",
                                "max_angular_accel",
                            )
                        },
                    }
                ],
            ),
        ]
    )

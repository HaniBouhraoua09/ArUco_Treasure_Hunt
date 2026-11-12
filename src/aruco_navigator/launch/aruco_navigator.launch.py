from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            DeclareLaunchArgument("marker_image_topic", default_value="/marker_image"),
            DeclareLaunchArgument(
                "marker_image_raw_topic", default_value="/marker_image_raw"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("min_detected_marker_side_px", default_value="12.0"),
            DeclareLaunchArgument("alignment_forward_speed", default_value="0.08"),
            DeclareLaunchArgument(
                "alignment_stop_error_fraction", default_value="0.15"
            ),
            DeclareLaunchArgument("alignment_near_target_ratio", default_value="0.8"),
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
                            )
                        },
                    }
                ],
            ),
        ]
    )

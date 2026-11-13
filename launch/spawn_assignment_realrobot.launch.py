import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_erl1 = get_package_share_directory('erl1')

    # 1. Launch Arguments for the Real ROSbot (use_sim_time MUST be false)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true (must be false on real ROSbot)'
    )
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz2 to monitor /marker_image and camera feed'
    )

    # Configurable ROSbot_1 Topic Arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/rosbot_1/camera/rgb/image_raw',
        description='Raw camera topic published by rosbot_1'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/rosbot_1/camera/rgb/camera_info',
        description='CameraInfo topic published by rosbot_1'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/rosbot_1/odometry/filtered',
        description='Odometry topic published by rosbot_1'
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/rosbot_1/cmd_vel',
        description='Velocity command topic for rosbot_1'
    )

    # 2. Optional RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_erl1, 'rviz', 'urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. Treasure Hunt Nodes
    perception_node = Node(
        package='erl1',
        executable='perception_node.py',
        name='perception_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),
        }]
    )

    visual_servoing_node = Node(
        package='erl1',
        executable='visual_servoing_node.py',
        name='visual_servoing_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }]
    )

    rotation_controller_node = Node(
        package='erl1',
        executable='rotation_controller_node.py',
        name='rotation_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }]
    )

    state_machine_node = Node(
        package='erl1',
        executable='state_machine_node.py',
        name='state_machine_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }]
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(use_sim_time_arg)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(image_topic_arg)
    launchDescriptionObject.add_action(camera_info_topic_arg)
    launchDescriptionObject.add_action(odom_topic_arg)
    launchDescriptionObject.add_action(cmd_vel_topic_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(perception_node)
    launchDescriptionObject.add_action(visual_servoing_node)
    launchDescriptionObject.add_action(rotation_controller_node)
    launchDescriptionObject.add_action(state_machine_node)

    return launchDescriptionObject
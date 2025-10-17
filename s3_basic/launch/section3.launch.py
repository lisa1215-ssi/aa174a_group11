from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    constant_control_node = Node(
            package='s3_basic',
            executable='constant_control.py'
        )

    # Launches RViz with your configuration file
    rviz_config = PathJoinSubstitution([FindPackageShare("s3_basic"), "rviz", "section3.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        constant_control_node,
        rviz_node
    ])

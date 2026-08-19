from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Use simulation time for all nodes
        Node(
            package='modelo_omnidireccional',
            executable='youbot_odometry_node',
            name='youbot_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package="modelo_omnidireccional",
            executable="logger",
            name="logger",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

    ])


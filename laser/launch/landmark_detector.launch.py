from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Pioneer odometry node
        Node(
            package="modelo_omnidireccional",
            executable="youbot_odometry_node",  # check executable name in ROS 2
            name="youbot_odometry",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),

        #Landmark detector
        Node(
            package="laser",
            executable="landmark_detector",
            name="landmark_detector",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
    ])
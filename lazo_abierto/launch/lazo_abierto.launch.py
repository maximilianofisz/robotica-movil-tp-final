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
            package='lazo_abierto',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'stepping': 0.1},
                {'trajectory_type': 'spline'},
                {'total_time': 50.0},
                {'amplitude': 1.0},
                {'cycles': 1.0},
                {'spline_waypoints': [
		    0.0,   0.0,  0.0,   -1.57,   # START AT ORIGIN
		    5.0,   1.0,  0.0,   -1.57,
		    10.0,  1.0, -2.0,    3.14,
		    15.0, -1.0, -2.0,    1.57,
		    20.0, -1.0,  0.0,    0.0,
		    25.0,  0.0,  0.0,   -1.57
		]}
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)

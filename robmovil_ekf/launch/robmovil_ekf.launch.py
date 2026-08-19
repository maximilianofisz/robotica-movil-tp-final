from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    debug = LaunchConfiguration('debug')
    detector = LaunchConfiguration('detector')

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='laser',
            executable='landmark_detector',
            name='landmark_detector',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link_ekf'},
                        {"use_sim_time": True}], 
            condition=IfCondition(detector)
        ),

        Node(
            package='laser',
            executable='landmark_detector',
            name='landmark_detector_gt',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link_gt'},
                        {"use_sim_time": True}], 
            remappings=[
                ('/landmarks_pointcloud', '/landmarks_pointcloud/groundtruth'),
                ('/landmarks', '/landmarks/groundtruth'),
            ],
            condition=IfCondition(detector)
        ),

        Node(
            package='modelo_omnidireccional',
            executable='youbot_odometry_node',
            name='youbot_odometry',
            output='screen',
            parameters=[{"use_sim_time": True}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{"use_sim_time": True}]
        ),

        # EKF node
        Node(
            package='robmovil_ekf',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[{'only_prediction': False},
                        {'min_landmark_size': 2},
                        {"use_sim_time": True}],
        ),

        # Nodo para logguear lo necesario para las metricas
        Node(
            package="robmovil_ekf",
            executable="logger",
            name="logger",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package="lazo_cerrado_ekf",
            executable="trajectory_follower_cl",
            name="trajectory_follower_cl",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {"goal_selection": "PURSUIT_BASED"}, #FIXED_GOAL, TIME_BASED, PURSUIT_BASED
                {"fixed_goal_x": float(2.0)},
                {"fixed_goal_y": float(2.0)},
                {"fixed_goal_a": float(-0.785)}, # -1/2 * PI
            ],
        ),

        # Creamos una trajectoria cuadrada mirando hacia afuera para usar los postes como referencias en EKF
        Node(
            package="lazo_abierto",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {'trajectory_type': 'spline'}, #sin or spline
                {"stepping": float(0.1)},
                {"total_time": float(20.0)},
                {"amplitude": float(1.0)},
                {"cycles": float(1.0)},
                {'spline_waypoints': [
                    0.0,   0.0,  1.0,   0.0,  
                    2.5,   1.0,  1.0,   0.0,
                    5.0,   1.0, 0.0,    0.0,
                    7.5,  1.0, -1.0,    0.0,
                    10.0,  0.0, -1.0,     0.0,
                    12.5, -1.0, -1.0,    0.0,
                    15.0, -1.0, 0.0,    0.0,
                    17.5, -1.0,  1.0,    0.0,
                    20.0,  0.0,  1.0,   0.0
                ]}
            ],  
        ),
    ])


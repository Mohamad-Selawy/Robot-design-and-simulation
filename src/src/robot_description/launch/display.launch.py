from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    #xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'my_urdf_robot_xacro.urdf.xacro'
    ])

    robot_description_content = Command(['xacro ', xacro_file])

    #path for my world
    world_file = '/usr/share/gazebo-11/worlds/cafe.world'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        # Start Gazebo Classic with cafe world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Start robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'robot_description': robot_description_content}
            ]
        ),

        # Spawn robot into Gazebo using spawn_entity
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-topic', 'robot_description',
                '-entity', 'my_robot'
            ],
            output='screen'
        )
    ])

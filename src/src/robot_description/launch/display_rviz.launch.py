from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to your xacro and RViz config files
    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'my_urdf_robot_xacro.urdf.xacro'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'rviz',
        'robot_config.rviz'
    ])

    # Generate robot_description from xacro
    robot_description_content = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # Joint State Publisher GUI for movable part
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Robot State Publisher for visual part
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'robot_description': robot_description_content}
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
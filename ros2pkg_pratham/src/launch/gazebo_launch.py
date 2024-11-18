import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare the world file path (adjust to your world file location)
        DeclareLaunchArgument('world', default_value='simple_world.world', description='Path to the Gazebo world'),
        
        # Launch Gazebo server with the world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '$(find gps_to_global_coordinates)/worlds/simple_world.world'],
        ),
        
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_client',
            output='screen',
        ),
        
        # Start your GPS-to-global-coordinates node
        Node(
            package='gps_to_global_coordinates',
            executable='gps_to_global_coordinates_node',
            name='gps_to_global_coordinates_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])


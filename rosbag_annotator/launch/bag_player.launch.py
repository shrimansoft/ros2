from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Path to the ROS bag directory'
    )
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback rate (1.0 = normal speed)'
    )
    
    # Bag player node
    bag_player_node = Node(
        package='rosbag_annotator',
        executable='bag_player.py',
        name='bag_player',
        output='screen',
        arguments=[LaunchConfiguration('bag_path'), LaunchConfiguration('playback_rate')]
    )
    
    return LaunchDescription([
        bag_path_arg,
        playback_rate_arg,
        bag_player_node,
    ])

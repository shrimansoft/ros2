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
    
    # Annotation GUI node
    annotation_gui_node = Node(
        package='rosbag_annotator',
        executable='annotation_gui.py',
        name='annotation_gui',
        output='screen',
        parameters=[{
            'bag_path': LaunchConfiguration('bag_path')
        }]
    )
    
    return LaunchDescription([
        bag_path_arg,
        annotation_gui_node,
    ])

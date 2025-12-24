from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('segmentation_3d')
    
    # Config file path
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')
    
    # Declare launch arguments
    camera_rgb_topic_arg = DeclareLaunchArgument(
        'camera_rgb_topic',
        default_value='/realsense/color/image_raw',
        description='Camera RGB image topic'
    )
    
    # Ultralytics segmentation node
    ultralytics_node = Node(
        package='segmentation_3d',
        executable='ultralytics_node',
        name='ultralytics_segmentation',
        output='log',
        prefix="bash -c 'exec > /dev/null 2>&1; exec \"$@\"' bash"
)
    
    # Segmentation processor node
    segmentation_processor_node = Node(
        package='segmentation_3d',
        executable='segmentation_processor_node',
        name='segmentation_processor_node',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        camera_rgb_topic_arg,
        ultralytics_node,
        segmentation_processor_node,
    ])


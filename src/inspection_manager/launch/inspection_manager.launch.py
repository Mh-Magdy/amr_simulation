from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    trucks_file = LaunchConfiguration("trucks_file")
    detection_topic = LaunchConfiguration("detection_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trucks_file",
                default_value="",
                description="Path to trucks YAML (falls back to package config).",
            ),
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/darknet_ros_3d/bounding_boxes",
                description="Topic for gb_visual_detection_3d_msgs/BoundingBoxes3d.",
            ),
            Node(
                package="inspection_manager",
                executable="inspection_manager_node",
                name="inspection_manager",
                output="screen",
                parameters=[
                    {"trucks_file": trucks_file},
                    {"detection_topic": detection_topic},
                ],
            ),
        ]
    )


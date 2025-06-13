from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('instance_segmentation')

    # Default paths
    default_model_path = os.path.join(pkg_share, 'models', 'model_final.pth')
    default_config_path = os.path.join(pkg_share, 'config', 'mask_rcnn_R_50_FPN.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to the model file'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to the config file'
        ),
        Node(
            package='instance_segmentation',
            executable='instance_segmentation_node',
            name='instance_segmentation_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'model_path': LaunchConfiguration('model_path'),
                'config_path': LaunchConfiguration('config_path')
            }]
        )
    ])
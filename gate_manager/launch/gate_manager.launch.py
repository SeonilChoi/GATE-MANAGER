import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("gate_manager"),
            "params",
            "gate_manager_parameters.yaml"
        )
    )

    motor_manager_node = Node(
        package="gate_manager",
        executable="motor_manager_node",
        parameters=[param_dir],
        output="screen",
        name="motor_manager_node"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "param_dir",
            default_value=param_dir,
            description="Path to the parameter file"
        ),
        motor_manager_node
    ])
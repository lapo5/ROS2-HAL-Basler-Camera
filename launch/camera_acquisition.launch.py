import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    params_acquisition_filepath = os.path.join(
        get_package_share_directory("hal_basler_camera"),
        "params",
        "params_camera_acquisition.yaml",
    )

    cam_acquisition_node = Node(
        package="hal_allied_vision_camera",
        executable="camera_acquisition",
        name="camera_acquisition",
        parameters=[params_acquisition_filepath],
    )

    return LaunchDescription([cam_acquisition_node])

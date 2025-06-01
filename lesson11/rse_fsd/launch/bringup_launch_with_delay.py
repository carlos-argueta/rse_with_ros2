# rse_fsd/launch/camera_node_config_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch the DashcamNode and load its parameters from a YAML file.
    """
    # Define the package name
    pkg_name = 'rse_fsd' # Replace with your actual package name

    # Construct the full path to the YAML parameter file
    # Assumes the 'config' directory is in the package's share directory
    class_count_config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'class_count_server_params.yaml'
    )

    camera_config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'dashcam_params2.yaml'
    )

    # Define the Node action for your dashcam node
    dashcam_node = TimerAction(
        period=12.0,  # Delay in seconds
        actions=[
            Node(
                package=pkg_name,
                executable='camera_node_params',
                name='dashcam_node',
                output='screen',
                # Pass a list containing the *path* to the YAML file to the parameters argument
                parameters=[camera_config_file_path]
            )
        ]
    )

  

    # Define the Node action for your dashcam node
    get_class_count_server = Node(
        package=pkg_name,
        executable='class_counts_server_params',
        name='get_class_count_server',
        output='screen',
        # Pass a list containing the *path* to the YAML file to the parameters argument
        parameters=[class_count_config_file_path]
    )

    # Create the LaunchDescription object and add the node action
    ld = LaunchDescription()
    ld.add_action(get_class_count_server)
    ld.add_action(dashcam_node)

    return ld
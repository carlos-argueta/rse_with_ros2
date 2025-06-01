# rse_fsd/launch/camera_node_configurable_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument # Import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration # Import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch DashcamNode, loading parameters from a YAML file specified
    as a launch argument, with a default file path.
    """
    # Define the package name
    pkg_name = 'rse_fsd' # Your package name

    # --- Begin Argument Declaration ---

    # 1. Construct the full path to the *default* YAML parameter file
    default_param_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'dashcam_params1.yaml' # This is the default file we expect
    )

    # 2. Declare the launch argument
    # This allows users to override the param_file path via the command line
    param_file_arg = DeclareLaunchArgument(
        name='param_file', # The name of the argument (used in command line)
        default_value=default_param_file_path, # The default value if not specified
        description='Full path to the ROS 2 parameters file to load for the dashcam node.'
    )

    # --- End Argument Declaration ---

    # --- Node Definition ---

    # 3. Use LaunchConfiguration to get the value of the argument
    # This will be the default path unless overridden by the user
    resolved_param_file = LaunchConfiguration('param_file')

    # Define the Node action for your dashcam node
    dashcam_node = Node(
        package=pkg_name,
        executable='camera_node_params',
        name='dashcam_node',
        output='screen',
        # Pass the LaunchConfiguration object (which resolves to the path)
        # inside a list to the parameters argument. The launch system handles
        # substituting the actual path string here.
        parameters=[resolved_param_file]
    )

    # --- Create Launch Description ---

    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Add the launch argument declaration *before* it's used (good practice)
    ld.add_action(param_file_arg)

    # Add the node that uses the argument's value
    ld.add_action(dashcam_node)

    return ld
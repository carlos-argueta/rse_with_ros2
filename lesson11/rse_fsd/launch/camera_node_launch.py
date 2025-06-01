# rse_fsd/launch/camera_node_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch the DashcamNode and set parameters directly in the launch file.
    """

    # Define the parameters you want to set for the node
    # This is a list of dictionaries, where each dictionary is {'param_name': value}
    node_params = [
        {'display_resolution': 1080},  # Set resolution to 1080p
        {'show_detections': False},   # Turn OFF detection display initially
        {'bbox_color': 'blue'}      # Set bounding box color to blue
    ]

    # Define the Node action for your dashcam node
    dashcam_node = Node(
        package='rse_fsd',            # Your package name
        executable='camera_node_params',    # The name of the executable entry point
        name='dashcam_node',          # The runtime name of the node
        output='screen',              # Show node output in the terminal
        parameters=node_params        # Pass the list of parameters here
    )

    # Create the LaunchDescription object and add the node action
    ld = LaunchDescription()
    ld.add_action(dashcam_node)

    return ld
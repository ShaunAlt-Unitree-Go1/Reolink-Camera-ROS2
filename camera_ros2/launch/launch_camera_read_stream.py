# =============================================================================
# Camera Reader + Streamer Launch
# Created by: Shaun Altmann (saltmann@deakin.edu.au)
# =============================================================================
'''
Camera Reader + Streamer Launch
-
Creates a new camera reader node, as well as an optional camera streamer node.
'''
# =============================================================================

# =============================================================================
# Imports
# =============================================================================

# used to define the structure of the launch file
from launch import LaunchDescription # type: ignore

# used to define arguments that can be passed to the launch file
from launch.actions import DeclareLaunchArgument # type: ignore

# used to log info messages from the launch file
from launch.actions import LogInfo # type: ignore

# used to define and configure ROS nodes to be launched
from launch_ros.actions import Node # type: ignore

# used for conditional logic based on launch arguments
from launch.conditions import IfCondition # type: ignore

# used to implement the values of launch arguments within this file
from launch.substitutions import LaunchConfiguration # type: ignore


# =============================================================================
# ROS2 Launch Description
# =============================================================================
def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # launch arguments
        DeclareLaunchArgument( # namespace of the nodes
            'namespace',
            description = (
                'Namespace (typically the camera name prefixed with a "/" (e.g. ' \
                + '"/Camera001").)'
            )
        ),
        DeclareLaunchArgument( # username of the camera
            'camera_uid',
            description = (
                'Username of the camera required to access the video stream.'
            )
        ),
        DeclareLaunchArgument( # password of the camera
            'camera_pwd',
            description = (
                'Password of the camera required to access the video stream.'
            )
        ),
        DeclareLaunchArgument( # ip of the camera
            'camera_ip',
            description = (
                'IP of the camera required to access the video stream.'
            )
        ),
        DeclareLaunchArgument( # flag for having a streamer
            'create_streamer',
            default_value = 'False',
            description = (
                'Flag for whether or not to create a streamer show the live ROS ' \
                + 'image message data. Defaults to "False".'
            ),
            choices = ['True', 'False']
        ),
        # node creation logger
        LogInfo( # log namespace
            msg = [
                'Launching Nodes in Namespace "',
                LaunchConfiguration('namespace'),
                '", Streamer Creation: ',
                LaunchConfiguration('create_streamer'),
            ]
        ),
        # camera reader node
        Node(
            package = 'camera_ros2',
            executable = 'camera_reader',
            namespace = LaunchConfiguration('namespace'),
            parameters = [{
                'camera_uid': LaunchConfiguration('camera_uid'),
                'camera_pwd': LaunchConfiguration('camera_pwd'),
                'camera_ip': LaunchConfiguration('camera_ip'),
            }]
        ),
        # camera streamer node
        Node(
            package = 'camera_ros2',
            executable = 'camera_streamer',
            namespace = LaunchConfiguration('namespace'),
            parameters = [{
                'title': LaunchConfiguration('namespace'),
            }],
            condition = IfCondition(LaunchConfiguration('create_streamer'))
        )
    ])


# =============================================================================
# End of File
# =============================================================================

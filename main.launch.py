from os import name, path, environ, getcwd

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


def generate_launch_description():
    chatbot = Node(
        package="chatbot", executable="chatbot_node", parameters=[{"name": "Will"}]
    )
    counter = Node(package="counter", executable="counter_node")
    arithmetic = Node(package="arithmetic", executable="arithmetic_node")
    rqt = Node(package="rqt_gui", executable="rqt_gui")

    # To stop a node from running, simply remove it here.
    return LaunchDescription([arithmetic, counter, chatbot, rqt])

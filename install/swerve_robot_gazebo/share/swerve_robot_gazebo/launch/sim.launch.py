import os
from typing import Final, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ROS Pkg Share
GAZEBO_PKG: Final[str] = "gazebo_ros"
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"

# File Names
GAZEBO_LAUNCH_FILE: Final[str] = "gzserver.launch.py"
SPAWN_LAUNCH_FILE: Final[str] = "spawn_swerve_robot.launch.py"

# Directory Names
LAUNCH_DIR: Final[str] = "launch"

# File Paths
gazebo_launch_path = os.path.join(
    get_package_share_directory(GAZEBO_PKG), LAUNCH_DIR, GAZEBO_LAUNCH_FILE
)
spawn_launch_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_GAZEBO_PKG), LAUNCH_DIR, SPAWN_LAUNCH_FILE
)

# Launch Arguments
ARGUMENTS: Optional[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    Generate Launch Description for Gazebo and Swerve Robot Spawn.

    Returns
    -------
        LaunchDescription: Description of the launch-able ROS system

    """
    
    # Start Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
    cmd=["gzclient"],
    output="screen"
    )



    # Start Gazebo using ROS Plugin
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
    )

    # Spawn Swerve Robot in Gazebo
    swerve_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_launch_path]),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(swerve_robot_spawn)
    ld.add_action(gazebo_client)

    return ld

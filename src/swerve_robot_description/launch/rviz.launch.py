import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# Packages
DESCRIPTION_PKG: Final[str] = "swerve_robot_description"

# Directories
URDF_DIR: Final[str] = "urdf"

# Files
ROBOT_XACRO_FILE: Final[str] = "swerve_robot.xacro"
RVIZ_CONFIG_FILE: Final[str] = "description.rviz"  # เพิ่มการอ้างอิงถึงไฟล์ RViz

# File Paths
description_path = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), URDF_DIR, ROBOT_XACRO_FILE
)

rviz_config_path = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), "rviz", RVIZ_CONFIG_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = [
    DeclareLaunchArgument(
        name="sim",
        default_value="false",
        choices=["true", "false"],
        description="Enable simulation control and sensor plugins",
    ),
]


def generate_launch_description() -> LaunchDescription:
    """
    1. Parse robot xacro file
    2. Launch robot state publisher for /robot_description and /tf
    3. Launch Joint State Publisher GUI
    4. Launch RViz with the robot model and configuration

    Launch Args:
    sim: false(default)
        - Loads gazebo control and sensor plugins
    """

    sim_flag = LaunchConfiguration("sim")

    robot_description = {
        "robot_description": Command(
            [
                "xacro --verbosity 0 ",
                description_path,
                " sim:=",
                sim_flag,
            ]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld

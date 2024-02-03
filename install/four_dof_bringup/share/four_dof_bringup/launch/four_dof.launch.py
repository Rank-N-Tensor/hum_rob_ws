# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os, xacro
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix


def generate_launch_description():
    package_description = "four_dof"
    package_directory = get_package_share_directory(package_description)
    install_dir_path = get_package_prefix(package_description) + "/share"
    robot_meshes_path = os.path.join(package_directory, "meshes/draft2")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["GAZEBO_RESOURCE_PATH"]:
                os.environ["GAZEBO_RESOURCE_PATH"] += ":" + resource_path
    else:
        os.environ["GAZEBO_RESOURCE_PATH"] = ":".join(gazebo_resource_paths)
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    """
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "verbose": "true",
        }.items(),
    )
    """

    """
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("four_dof"),
                    "urdf",
                    "urdf.xacro",
                ]
            ),
            " ",
            "use_gazebo_classic:=true",
        ]
    )
    """

    bot_description_path = os.path.join(get_package_share_directory("four_dof"))

    xacro_file = os.path.join(bot_description_path, "urdf", "draft2.xacro")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    robot_description = {"robot_description": robot_description_config}

    """
    urdf_file_name = "draft2.urdf"  # Replace with your URDF file name
    urdf_file_path = os.path.join(bot_description_path, "urdf", urdf_file_name)

    # Robot Description Configuration
    robot_description_config = open(urdf_file_path).read()
    robot_description = {"robot_description": robot_description_config}
    """
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("four_dof_bringup"),
            "config",
            "controller_config.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("four_dof"), "rviz", "modified_4_dof.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "four_dof_bot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.25",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delay_joint_state_broadcaster_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    """
    delay_spawn_entity_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[spawn_entity],
        )
    )
    """

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        control_node,
        spawn_entity,
        robot_state_pub_node,
        delay_joint_state_broadcaster_after_spawn_entity,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

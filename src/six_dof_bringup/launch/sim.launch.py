import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/joint_trajectory_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            bridge,
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("ros_ign_gazebo"),
                            "launch",
                            "ign_gazebo.launch.py",
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [" -r -v 4 empty.sdf"])]),
                # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),])
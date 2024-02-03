import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_file = "6dof.urdf"
    package_description = "six_dof"
    print("Fetching URDF ==>")

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "urdf", urdf_file
    )

    # nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(["xacro ", robot_desc_path]),
            }
        ],
        output="screen",
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description), "rviz", "modified_4_dof.rviz"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",
        parameters=[{"use_sim_time": True}],
        #
        # arguments=(["-d", rviz_config_dir]),
    )

    # LaunchDescription obj
    return LaunchDescription([robot_state_publisher_node, rviz2_node])

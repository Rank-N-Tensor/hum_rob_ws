import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node,SetParameter


def generate_launch_description():
    # you need to modify the resource path for gazebo to find the models
    package_description = "eight_dof"
    package_directory = get_package_share_directory(package_description)
    install_dir_path = get_package_prefix(package_description) + "/share"
    robot_meshes_path = os.path.join(package_directory, "meshes")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += ":" + resource_path
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(gazebo_resource_paths)

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    pkg_path = os.path.join(get_package_share_directory("eight_dof"))

    urdf_file = os.path.join(pkg_path, "urdf", "simplified_8dof_from_hip.urdf")
    #urdf_file = os.path.join(pkg_path, "urdf", "8dof_from_hip.urdf")

    with open(urdf_file, "r") as inf:
        robot_description_content = inf.read()

    params = {"robot_description": robot_description_content}

    # six_dof_bringup_dir = get_package_share_directory('six_dof_bringup')
    # controller_config_path = os.path.join(six_dof_bringup_dir, 'config', 'controller_config.yaml')

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "8dof",  
            "-allow_renaming",
            "true",
            "-x",
            "0",  # X position in meters
            "-y",
            "0",  # Y position in meters
            "-z",
            "0.5",  # Z position in meters (adjust this to bring the robot to the ground)
            "-R",
            "0",  # Roll in radians
            "-P",
            "0",  # Pitch in radians
            "-Y",
            "0",
        ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/joint_trajectory_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            "/left_contact@ros_gz_interfaces/msg/Contacts@ignition.msgs.Contacts",
            "/right_contact@ros_gz_interfaces/msg/Contacts@ignition.msgs.Contacts"
        ],
        output="screen",
    )
    return LaunchDescription(

        [
            SetParameter(name='use_sim_time', value=use_sim_time),  #SetParameter sets the specified parameter for all nodes
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
                launch_arguments=[("gz_args", [" -r -v 4 empty.sdf"])],
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=ignition_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            node_robot_state_publisher,
            ignition_spawn_entity,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )

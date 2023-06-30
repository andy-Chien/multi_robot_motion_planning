# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import yaml


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ns = LaunchConfiguration("ns")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_port = LaunchConfiguration("reverse_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    script_command_port = LaunchConfiguration("script_command_port")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    pose_xyz = LaunchConfiguration("pose_xyz")
    pose_rpy = LaunchConfiguration("pose_rpy")
    multi_arm = LaunchConfiguration("multi_arm")
    calib_file = LaunchConfiguration("calib_file")


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("mr_description"), "config", "universal_robots", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("mr_description"), "config/universal_robots", ur_type, calib_file]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mr_description"), "urdf", "universal_robots", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "reverse_port:=",
            reverse_port,
            " ",
            "trajectory_port:=",
            trajectory_port,
            " ",
            "script_sender_port:=",
            script_sender_port,
            " ",
            "script_command_port:=",
            script_command_port,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_rx_idle_chars:=",
            tool_rx_idle_chars,
            " ",
            "tool_tx_idle_chars:=",
            tool_tx_idle_chars,
            " ",
            "tool_device_name:=",
            tool_device_name,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "pose_xyz:=",
            pose_xyz,
            " ",
            "pose_rpy:=",
            pose_rpy,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # initial_joint_controllers = PathJoinSubstitution(
    #     [FindPackageShare(runtime_config_package), "config", controllers_file]
    # )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("mr_config"), "rviz", "view_robot.rviz"]
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("mr_config"),
            "config", "universal_robots",
            ur_type.perform(context) + "_update_rate.yaml",
        ]
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("mr_config"), "config", "universal_robots", "ur_controllers.yaml"]
    )
    ns_text = ns.perform(context)
    prefix_text = prefix.perform(context)
    if ns_text != "" or prefix_text != "":
        ros2_controllers_yaml = load_yaml("mr_config", "config/universal_robots/ur_controllers.yaml")
        if prefix_text != "":
            for v in ros2_controllers_yaml.values():
                if 'joints' not in v['ros__parameters']:
                    continue
                if 'constraints' in v['ros__parameters']:
                    for j_name in v['ros__parameters']["joints"]:
                        v['ros__parameters']['constraints'][prefix_text + j_name] = \
                            v['ros__parameters']['constraints'][j_name]
                        del v['ros__parameters']['constraints'][j_name]
                v['ros__parameters']["joints"] = [prefix_text + j for j in v['ros__parameters']["joints"]]
        if ns_text != "":
            ros2_controllers_yaml = {ns_text: ros2_controllers_yaml}

        file_name = ns_text + prefix_text + "ur_controllers.yaml"
        file_out = get_package_share_directory("mr_config") + "/config/universal_robots/" + file_name
        with open(file_out, "w") as file_out:
            file_out.write(yaml.dump(ros2_controllers_yaml))
        ros2_controllers_path = PathJoinSubstitution(
            [FindPackageShare("mr_config"), "config", "universal_robots", file_name]
        )

    multi_arm_text = multi_arm.perform(context)
    use_fake_hardware_text = use_fake_hardware.perform(context)
    joint_trajectory_controller_to_spawn = "scaled_joint_trajectory_controller"
    if multi_arm_text == "true":
        joint_trajectory_controller_to_spawn = "mr_joint_trajectory_controller"
    elif use_fake_hardware_text == "true":
        joint_trajectory_controller_to_spawn = "joint_trajectory_controller"

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, update_rate_config_file, ros2_controllers_path],
        output="screen",
        namespace=ns,
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[robot_description, update_rate_config_file, ros2_controllers_path],
        output="screen",
        namespace=ns,
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(launch_dashboard_client) and UnlessCondition(use_fake_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        namespace=ns,
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        namespace=ns,
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": tool_device_name,
            }
        ],
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        namespace=ns,
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=ns,
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        namespace=ns,
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "controller_manager"
        ],
    )

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["io_and_status_controller", "-c", "controller_manager"],
    )

    speed_scaling_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "speed_scaling_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "controller_manager",
        ],
    )

    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "controller_manager",
        ],
    )

    forward_position_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "forward_position_controller",
            "--controller-manager-timeout",
            "300",
            "-c", "controller_manager",
            "--inactive"
        ],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[joint_trajectory_controller_to_spawn, "-c", "controller_manager"],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[joint_trajectory_controller_to_spawn, "-c", "controller_manager", "--inactive"],
        condition=UnlessCondition(activate_joint_controller),
    )

    nodes_to_start = [
        control_node,
        ur_control_node,
        dashboard_client_node,
        tool_communication_node,
        controller_stopper_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        speed_scaling_state_broadcaster_spawner,
        force_torque_sensor_broadcaster_spawner,
        forward_position_controller_spawner_stopped,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="name of namespace",
        )
    )
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Reverse port by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Trajectory points command port."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="Script sender port by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Script command port by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_robot_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "multi_arm",
            default_value="false",
            description="Indicate running with multi robot or not.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. \
            The user has be be allowed to write to this location. \
            Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. \
            Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_xyz", 
            default_value='"0 0 0"', 
            description="position of robot in world",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_rpy", 
            default_value='"0 0 0"', 
            description="orientation of robot in world",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "calib_file",
            default_value="default_kinematics.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

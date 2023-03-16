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

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ns = LaunchConfiguration("ns")
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_controller = LaunchConfiguration("use_fake_controller")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    launch_servo = LaunchConfiguration("launch_servo")
    pose_xyz = LaunchConfiguration("pose_xyz")
    pose_rpy = LaunchConfiguration("pose_rpy")
    multi_arm = LaunchConfiguration("multi_arm")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
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
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
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

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # robot_description_planning = {
    # "robot_description_planning": load_yaml_abs(str(joint_limit_params.perform(context)))
    # }

    capabilities_to_disable = "move_group/MoveGroupCartesianPathService "
    capabilities_to_disable += "move_group/MoveGroupPlanService "
    # capabilities_to_disable += "move_group/MoveGroupQueryPlannersService "
    capabilities_to_disable += "move_group/MoveGroupStateValidationService "
    # capabilities_to_disable += "move_group/MoveGroupGetPlanningSceneService "
    # capabilities_to_disable += "move_group/ApplyPlanningSceneService "
    capabilities_to_disable += "move_group/ClearOctomapService "
    capabilities_to_disable += "move_group/MoveGroupExecuteTrajectoryAction"

    capabilities_to_enable = "moveit_capability/MoveGroupAsyncExecuteTrajectoryAction"

    move_group_acapabilities_setting = {
        "disable_capabilities": capabilities_to_disable,
        "capabilities": capabilities_to_enable
    }

    planning_adapters = "planning_adapter/AddTrajectoryObstacles "
    planning_adapters += "default_planner_request_adapters/AddTimeOptimalParameterization "
    planning_adapters += "default_planner_request_adapters/FixWorkspaceBounds "
    planning_adapters += "default_planner_request_adapters/FixStartStateBounds "
    planning_adapters += "default_planner_request_adapters/FixStartStateCollision "
    planning_adapters += "default_planner_request_adapters/FixStartStatePathConstraints"

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlannerC",
            "request_adapters": planning_adapters,
            "start_state_max_bounds_error": 0.1,
            "extra_robot_padding": 0.015,
        }
    }
    prefix_text = prefix.perform(context)
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    ompl_planning_pipeline_config["move_group"]["planner_configs"]["AdaptPRMkDefault"] \
        ["planner_data_path"] += prefix_text + 'adapt_prm.graph'

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    use_fake_hardware_text = use_fake_hardware.perform(context)
    multi_arm_text = multi_arm.perform(context)

    joint_trajectory_controller_to_spawn = "scaled_joint_trajectory_controller"
    if multi_arm_text == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["mr_joint_trajectory_controller"]["default"] = True
        joint_trajectory_controller_to_spawn = "mr_joint_trajectory_controller"
    elif use_fake_hardware_text == "true" and multi_arm_text == "false":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True
        joint_trajectory_controller_to_spawn = "joint_trajectory_controller"

    if prefix_text != "":
        controllers_yaml["scaled_joint_trajectory_controller"]["joints"] = \
            [prefix_text + x for x in controllers_yaml["scaled_joint_trajectory_controller"]["joints"]]
        controllers_yaml["joint_trajectory_controller"]["joints"] = \
            [prefix_text + x for x in controllers_yaml["joint_trajectory_controller"]["joints"]]
        controllers_yaml["mr_joint_trajectory_controller"]["joints"] = \
            [prefix_text + x for x in controllers_yaml["mr_joint_trajectory_controller"]["joints"]]

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        namespace=ns,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_acapabilities_setting,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", rviz_config_file]
    )
    rviz_node = Node(
        namespace=ns,
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            # robot_description_planning,
            warehouse_ros_config,
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        namespace=ns,
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    # Static TF
    # static_tf = Node(
    #     namespace=ns,
    #     package="tf2_ros",
    #     condition=IfCondition(use_fake_controller),
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # Publish TF
    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        condition=IfCondition(use_fake_controller),
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
    )
    ns_text = ns.perform(context)
    ros2_controllers_path_text = ros2_controllers_path.perform(context)
    if prefix_text != "":
        ros2_controllers_yaml = load_yaml("ur_robot_driver", "config/ur_controllers.yaml")
        jtc = 'joint_trajectory_controller'
        rp = 'ros__parameters'
        ros2_controllers_yaml[ns_text] = dict()
        ros2_controllers_yaml[ns_text][jtc] = \
            ros2_controllers_yaml[jtc]
        ros2_controllers_yaml[ns_text]["scaled_" + jtc] = \
            ros2_controllers_yaml["scaled_" + jtc]
        ros2_controllers_yaml[ns_text]["mr_" + jtc] = \
            ros2_controllers_yaml["mr_" + jtc]
        ros2_controllers_yaml[ns_text][jtc][rp]["joints"] = \
            [prefix_text + j for j in ros2_controllers_yaml[ns_text][jtc][rp]["joints"]]
        ros2_controllers_yaml[ns_text]["scaled_" + jtc][rp]["joints"] = \
            [prefix_text + j for j in ros2_controllers_yaml[ns_text]["scaled_" + jtc][rp]["joints"]]
        ros2_controllers_yaml[ns_text]["mr_" + jtc][rp]["joints"] = \
            [prefix_text + j for j in ros2_controllers_yaml[ns_text]["mr_" + jtc][rp]["joints"]]

        ros2_controllers_yaml['/**'] = None
        del ros2_controllers_yaml['/**']
        del ros2_controllers_yaml[jtc]
        del ros2_controllers_yaml["scaled_" + jtc]
        del ros2_controllers_yaml["mr_" + jtc]

        with open(ros2_controllers_path_text, "r") as file_in:
            file_in_text = file_in.read()
            file_out = get_package_share_directory("ur_robot_driver") + \
                "/config/" + prefix_text + "ur_controllers.yaml"
            with open(file_out, "w") as file_out:
                file_out.write(file_in_text + yaml.dump(ros2_controllers_yaml))
        ros2_controllers_path = PathJoinSubstitution(
            [FindPackageShare("ur_robot_driver"), "config", prefix_text + "ur_controllers.yaml"]
        )

    ros2_control_node = Node(
        namespace=ns,
        package="controller_manager",
        condition=IfCondition(use_fake_controller),
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            ros2_controllers_path,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        namespace=ns,
        package="controller_manager",
        condition=IfCondition(use_fake_controller),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        namespace=ns,
        package="controller_manager",
        condition=IfCondition(use_fake_controller),
        executable="spawner",
        arguments=[joint_trajectory_controller_to_spawn, "-c", "controller_manager"],
    )

    nodes_to_start = [
        # static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node, 
        rviz_node, 
        servo_node
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
            "use_fake_hardware",
            default_value="true",
            description="Indicate whether robot is running with fake hardware in simulator.",
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
            "use_fake_controller",
            default_value="true",
            description="Indicate whether robot is running with fake controller.",
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
            "description_package",
            default_value="mr_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
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
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for \
        the trajectory planing in simulation.",
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
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="view_robot.rviz",
            description="rviz description file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo", 
            default_value="false", 
            description="Launch Servo?",
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

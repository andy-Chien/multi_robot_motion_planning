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
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    arm_prefix = LaunchConfiguration("arm_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    launch_servo = LaunchConfiguration("launch_servo")
    pose_xyz = LaunchConfiguration("pose_xyz")
    pose_rpy = LaunchConfiguration("pose_rpy")
    multi_arm = LaunchConfiguration("multi_arm")
    calib_file = LaunchConfiguration("calib_file")
    description_file = LaunchConfiguration("description_file")
    srdf_file = LaunchConfiguration("srdf_file")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("mr_description"), "config", "universal_robots", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("mr_description"), "config", "universal_robots", ur_type, calib_file]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    print('description_file = {}'.format(description_file.perform(context)))

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mr_description"), "urdf", description_file]),
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
            "arm_prefix:=",
            arm_prefix,
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
                [FindPackageShare("mr_config"), "srdf", srdf_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "arm_prefix:=",
            arm_prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("mr_config"), "config", "moveit", "kinematics.yaml"]
    )

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

    multi_arm_text = multi_arm.perform(context)
    planning_adapters = ""
    if multi_arm_text == "true":
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
            "extra_robot_padding": 0.03,
        }
    }
    prefix_text = arm_prefix.perform(context)
    ompl_planning_yaml = load_yaml("mr_config", "config/moveit/ompl_planning.yaml")
    ompl_planning_yaml["planner_configs"]["AdaptPRMkDefault"] \
        ["planner_data_path"] += prefix_text + 'adapt_prm.graph'
    ompl_planning_yaml["planner_configs"]["AdaptLazyPRMkDefault"] \
        ["planner_data_path"] += prefix_text + 'adapt_lazy_prm.graph'
    if 'projection_evaluator_joints' in ompl_planning_yaml["ur_manipulator"].keys():
        pej = ompl_planning_yaml["ur_manipulator"]["projection_evaluator_joints"]
        pe = 'joints('
        for j in pej:
            pe += prefix_text + j + ','
        ompl_planning_yaml["ur_manipulator"]['projection_evaluator'] = pe[:-1] + ')'
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("mr_config", "config/moveit/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    use_fake_hardware_text = use_fake_hardware.perform(context)

    joint_trajectory_controller_to_spawn = "scaled_joint_trajectory_controller"
    if multi_arm_text == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["mr_joint_trajectory_controller"]["default"] = True
        joint_trajectory_controller_to_spawn = "mr_joint_trajectory_controller"
    elif use_fake_hardware_text == "true":
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
        # arguments=['--ros-args', '--log-level', "debug"]
    )

    # rviz with moveit configuration
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mr_config"), "rviz", rviz_config_file]
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
    servo_yaml = load_yaml("mr_config", "config/moveit/ur_servo.yaml")
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
    # marker_static_tf = Node(
    #     namespace=ns,
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "world_marker"],
    # )

    # camera_static_tf = Node(
    #     namespace=ns,
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["-0.03433662029772863", 
    #                 "-0.0033561763879540957",
    #                 "0.03618988925330751", 
    #                 "0.257812563704949", 
    #                 "0.113033449678485", 
    #                 "-0.382062384987442", 
    #                 "-0.880218413365325", 
    #                 "tool0", 
    #                 "camera_link"],
    # )
    # camera_static_tf = Node(
    #     namespace=ns,
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["-0.23927503063158623", 
    #                "0.13672661802295893",
    #                "-0.07798876382208922",
    #                " -0.3571993783034698",
    #                " -0.34977644374826966",
    #                " 0.6006152424568505", 
    #                " 0.6239602343634785", 
    #                "base_link", 
    #                "camera_link"],
    # )

    camera_static_tf = Node(
        namespace=ns,
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.2920878057867339", 
                   "-0.15407630829656666",
                   "-0.04049023141189538",
                   "-0.344249054489911",
                   "0.354365325505531",
                   "-0.616435762119046", 
                   "0.6131270306738", 
                   "base_link", 
                   "camera_link"],
    )
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
        [FindPackageShare("mr_config"), "config", "universal_robots", "ur_controllers.yaml"]
    )
    ns_text = ns.perform(context)
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
        # marker_static_tf,
        # camera_static_tf,
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
            "arm_prefix",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "calib_file",
            default_value="default_kinematics.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="universal_robots/ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "srdf_file",
            default_value="universal_robots/ur.srdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ur_moveit_config.launch_common import load_yaml

#robot parameters
RP = {
    'robot_1': {
        'ur_type': 'ur10e',
        'prefix': 'robot_1_',
        'pose_xyz': '"-0.5 0.8 0.1"',
        'pose_rpy': '"0.0 0.0 -1.57"'
    },
    'robot_2': {
        'ur_type': 'ur10e',
        'prefix': 'robot_2_',
        'pose_xyz': '"-0.8 -0.5 0.1"',
        'pose_rpy': '"0.0 0.0 0.0"' 
    },
    'robot_3': {
        'ur_type': 'ur10e',
        'prefix': 'robot_3_',
        'pose_xyz': '"0.8 0.5 0.0"',
        'pose_rpy': '"0.0 0.0 3.14"'
    },
    'robot_4': {
        'ur_type': 'ur10e',
        'prefix': 'robot_4_',
        'pose_xyz': '"0.5 -0.8 0.1"',
        'pose_rpy': '"0.0 0.0 1.57"' 
    }
}


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")


    rviz_params = []
    object_to_start = []

    for rn in RP: # rn: robot_name
        joint_limit_params = PathJoinSubstitution(
            [FindPackageShare("mr_description"), "config", "universal_robots", RP[rn]['ur_type'], "joint_limits.yaml"]
        )
        kinematics_params = PathJoinSubstitution(
            [FindPackageShare("ur_description"), "config", RP[rn]['ur_type'], "default_kinematics.yaml"]
        )
        physical_params = PathJoinSubstitution(
            [FindPackageShare("ur_description"), "config", RP[rn]['ur_type'], "physical_parameters.yaml"]
        )
        visual_params = PathJoinSubstitution(
            [FindPackageShare("ur_description"), "config", RP[rn]['ur_type'], "visual_parameters.yaml"]
        )

        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare("mr_description"), "urdf", "universal_robots", "ur.urdf.xacro"]),
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
                "safety_limits:=true",
                " ",
                "name:=",
                "ur",
                " ",
                "ur_type:=",
                RP[rn]['ur_type'],
                " ",
                "prefix:=",
                RP[rn]['prefix'],
                " ",
                "pose_xyz:=",
                RP[rn]['pose_xyz'],
                " ",
                "pose_rpy:=",
                RP[rn]['pose_rpy'],
                " ",
            ]
        )
        robot_description = {RP[rn]['prefix'] + "description": robot_description_content}

        # MoveIt Configuration
        robot_description_semantic_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("mr_config"), "srdf", "universal_robots", "ur.srdf.xacro"]
                ),
                " ",
                "name:=ur",
                " ",
                "prefix:=",
                RP[rn]['prefix'],
                " ",
            ]
        )
        robot_description_semantic = {
            RP[rn]['prefix'] + "description_semantic": robot_description_semantic_content}
        
        kinematics_yaml = load_yaml("mr_config", "config/moveit/kinematics.yaml")

        robot_description_kinematics = {
            RP[rn]['prefix'] + "description_kinematics": \
                kinematics_yaml['/**']['ros__parameters']['robot_description_kinematics']}
        rviz_params.append(robot_description)
        rviz_params.append(robot_description_semantic)
        rviz_params.append(robot_description_kinematics)

        launch_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("mr_config"), "/launch", "/ur_moveit.launch.py"]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "ur_type": RP[rn]['ur_type'],
                "ns": rn,
                "prefix": RP[rn]['prefix'],
                "rviz_config_file": rn + ".rviz",
                "pose_xyz": RP[rn]['pose_xyz'],
                "pose_rpy": RP[rn]['pose_rpy'],
                "multi_arm": "true",
                "launch_rviz": "false"
            }.items(),
        )
        object_to_start.append(launch_moveit)
        

    ompl_planning_yaml = load_yaml("mr_config", "config/moveit/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    # rviz with moveit configuration
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mr_config"), "rviz", "four_robot.rviz"]
    )
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "~/.ros/warehouse_ros.sqlite",
    }
    rviz_params.append(ompl_planning_pipeline_config)
    rviz_params.append(warehouse_ros_config)

    rviz_node = Node(
        namespace='',
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_params,
    )
    print("rviz_params = {}".format(rviz_params))
    object_to_start.append(rviz_node)

    return object_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_1",
            default_value="false",
            description="Launch robot 1 or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_2",
            default_value="false",
            description="Launch robot 2 or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Using sim time or not",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
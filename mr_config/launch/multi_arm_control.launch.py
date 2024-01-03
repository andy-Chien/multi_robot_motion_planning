from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ur_moveit_config.launch_common import load_yaml

#robot parameters
#  '"0.21502 0.78661 -0.0766"'
#  [ -0.0125393, 0.0059258, 0.0176579, 0.9997479 ] xyzw
#  '"-0.58768 0.26583 -014994"'
#  [ -0.0022718, 0.0030527, 0.92198, 0.3872188 ] xyzw
RP = {
    'robot_1': {
        'ur_type': 'ur5e',
        'arm_prefix': 'robot_1_',
        'pose_xyz': '"0.18610747 0.79149527 -0.08777093"',
        'pose_rpy': '"-0.025286 0.0114061 0.0354652"',
        'robot_ip': '192.168.1.100',
        'reverse_port': '50005',
        'trajectory_port': '50007',
        'script_sender_port': '50006',
        'script_command_port': '50008'
    },
    'robot_2': {
        'ur_type': 'ur5',
        'arm_prefix': 'robot_2_',
        'pose_xyz': '"0 0.7 0"',
        'pose_rpy': '"0 0 -1.5707963"',
        'robot_ip': '192.168.1.100',
        'reverse_port': '50001',
        'trajectory_port': '50003',
        'script_sender_port': '50002',
        'script_command_port': '50004'
    }
}


def launch_setup(context, *args, **kwargs):
    launch_robot_1 = LaunchConfiguration("robot_1")
    launch_robot_2 = LaunchConfiguration("robot_2")
    tool_changeable = LaunchConfiguration("tool_changeable")

    urdf_file = "ur.urdf.xacro"

    if tool_changeable.perform(context) == "true":
        urdf_file = "ur_tool_changeable.urdf.xacro"
    if launch_robot_1.perform(context) == "false":
        del RP['robot_1']
    if launch_robot_2.perform(context) == "false":
        del RP['robot_2']
    object_to_start = []
    launch_delay = 0.0
    for rn in RP: # rn: robot_name
        launch_ur_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("mr_config"), "/launch", "/ur_control.launch.py"]
            ),
            launch_arguments={
                "use_sim_time": 'false',
                "ur_type": RP[rn]['ur_type'],
                "ns": rn,
                "robot_ip": RP[rn]['robot_ip'],
                "reverse_port": RP[rn]['reverse_port'],
                "trajectory_port": RP[rn]['trajectory_port'],
                "script_sender_port": RP[rn]['script_sender_port'],
                "script_command_port": RP[rn]['script_command_port'],
                "arm_prefix": RP[rn]['arm_prefix'],
                "rviz_config_file": rn + ".rviz",
                "pose_xyz": RP[rn]['pose_xyz'],
                "pose_rpy": RP[rn]['pose_rpy'],
                "multi_arm": "true",
                "launch_rviz": "false",
                "initial_joint_controller": "mr_joint_trajectory_controller",
                "description_file": urdf_file
            }.items(),
        )
        
        object_to_start.append(TimerAction(
            period=launch_delay,
            actions=[launch_ur_control]
        ))
        launch_delay += 8
    print(object_to_start)
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
            "tool_changeable", 
            default_value="false", 
            description="Use tool changeable setting?",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
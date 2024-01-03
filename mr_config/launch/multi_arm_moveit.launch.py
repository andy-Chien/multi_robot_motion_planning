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

# rviz random test
RP = {
    # 'robot_1': {
    #     'ur_type': 'ur5e',
    #     'arm_prefix': 'robot_1_',
    #     'pose_xyz': '"0 -0.7 0"',
    #     'pose_rpy': '"0 0 1.5707963"',
    #     'description_pkg': "mr_description",
    #     'config_pkg': "mr_config",
    #     'moveit_launch_file': "ur_moveit.launch.py",
    #     'urdf_file': 'universal_robots/ur_tool_changeable.urdf.xacro',
    #     'srdf_file': 'universal_robots/ur_tool_changeable.srdf.xacro',
    #     'planner_config_name': 'ur',
    # },
    'robot_1': {
        'ur_type': 'ur5e',
        'arm_prefix': 'robot_1_',
        'pose_xyz': '"0 -0.7 0"',
        'pose_rpy': '"0 0 1.5707963"',
        'description_pkg': "mm_description",
        'config_pkg': "mm_moveit_config",
        'moveit_launch_file': "mm_moveit_with_fake_controller.launch.py",
        'urdf_file': 'mm_tool_changeable.urdf.xacro',
        'srdf_file': 'mm_tool_changeable.srdf.xacro',
        'planner_config_name': 'mobile_manipulator',
    },
    'robot_2': {
        'ur_type': 'ur5',
        'arm_prefix': 'robot_2_',
        'pose_xyz': '"0 0.7 0"',
        'pose_rpy': '"0 0 -1.5707963"',
        'description_pkg': "mr_description",
        'config_pkg': "mr_config",
        'moveit_launch_file': "ur_moveit.launch.py",
        'urdf_file': 'universal_robots/ur_tool_changeable.urdf.xacro',
        'srdf_file': 'universal_robots/ur_tool_changeable.srdf.xacro',
        'planner_config_name': 'ur',
    },
}


#  '"0.21502 0.78661 -0.0766"'
#  [ -0.0125393, 0.0059258, 0.0176579, 0.9997479 ] xyzw
#  '"-0.58768 0.26583 -014994"'
#  [ -0.0022718, 0.0030527, 0.92198, 0.3872188 ] xyzw

# ntu real robot test
# RP = {
#     'robot_1': {
#         'ur_type': 'ur5',
#         'arm_prefix': 'robot_1_',
#         'pose_xyz': '"0.18610747 0.79149527 -0.09877093"',
#         'pose_rpy': '"-0.025286 0.0114061 0.0354652"'
#     },
#     'robot_2': {
#         'ur_type': 'ur10e',
#         'arm_prefix': 'robot_2_',
#         'pose_xyz': '"0.2218704 -0.60678568 -0.1450561"',
#         'pose_rpy': '"-0.0073885 -0.001825 2.34635345"' 
#     }
# }


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_robot_1 = LaunchConfiguration("robot_1")
    launch_robot_2 = LaunchConfiguration("robot_2")
    tool_changeable = LaunchConfiguration("tool_changeable")
    urdf_file = "ur.urdf.xacro"
    srdf_file = "ur.srdf.xacro"
    rviz_file = "multi_robot_1.rviz"

    if tool_changeable.perform(context) == "true":
        urdf_file = "ur_tool_changeable.urdf.xacro"
        srdf_file = "ur_tool_changeable.srdf.xacro"
    if launch_robot_1.perform(context) == "false":
        del RP['robot_1']
        rviz_file = "robot_2.rviz"
    if launch_robot_2.perform(context) == "false":
        del RP['robot_2']
        rviz_file = "robot_1.rviz"

    rviz_params = []
    object_to_start = []

    for rn in RP: # rn: robot_name
        print('RPRPRPRPRPRPRPRP = {}, rviz_file = {}'.format(RP, rviz_file))
        joint_limit_params = PathJoinSubstitution(
            [FindPackageShare(RP[rn]['description_pkg']), "config", "universal_robots", RP[rn]['ur_type'], "joint_limits.yaml"]
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
                PathJoinSubstitution([FindPackageShare(RP[rn]['description_pkg']), "urdf", RP[rn]['urdf_file']]),
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
                RP[rn]['planner_config_name'],
                " ",
                "ur_type:=",
                RP[rn]['ur_type'],
                " ",
                "arm_prefix:=",
                RP[rn]['arm_prefix'],
                " ",
                "pose_xyz:=",
                RP[rn]['pose_xyz'],
                " ",
                "pose_rpy:=",
                RP[rn]['pose_rpy'],
                " ",
                "arm_type:=",
                RP[rn]['ur_type'],
                " ",
            ]
        )
        robot_description = {RP[rn]['arm_prefix'] + "description": robot_description_content}

        # MoveIt Configuration
        robot_description_semantic_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(RP[rn]['config_pkg']), "srdf", RP[rn]['srdf_file']]
                ),
                " ",
                "name:=",
                RP[rn]['planner_config_name'],
                " ",
                "arm_prefix:=",
                RP[rn]['arm_prefix'],
                " ",
            ]
        )
        robot_description_semantic = {
            RP[rn]['arm_prefix'] + "description_semantic": robot_description_semantic_content}
        
        kinematics_yaml = load_yaml("mr_config", "config/moveit/kinematics.yaml")

        robot_description_kinematics = {
            RP[rn]['arm_prefix'] + "description_kinematics": \
                kinematics_yaml['/**']['ros__parameters']['robot_description_kinematics']}
        rviz_params.append(robot_description)
        rviz_params.append(robot_description_semantic)
        rviz_params.append(robot_description_kinematics)

        launch_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare(RP[rn]['config_pkg']), "/launch/", RP[rn]['moveit_launch_file']]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "ur_type": RP[rn]['ur_type'],
                "arm_type": RP[rn]['ur_type'],
                "ns": rn,
                "arm_prefix": RP[rn]['arm_prefix'],
                "rviz_config_file": rn + ".rviz",
                "pose_xyz": RP[rn]['pose_xyz'],
                "pose_rpy": RP[rn]['pose_rpy'],
                "multi_arm": "true",
                "launch_rviz": "false",
                "description_file": RP[rn]['urdf_file'],
                "srdf_file": RP[rn]['srdf_file'],
            }.items(),
        )
        object_to_start.append(launch_moveit)
        
    static_tf = Node(
        # namespace=ns,
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "map"],
    )
    object_to_start.append(static_tf)

    ompl_planning_yaml = load_yaml("mr_config", "config/moveit/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    # rviz with moveit configuration
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mr_config"), "rviz", rviz_file]
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
    object_to_start.append(rviz_node)



    return object_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_1",
            default_value="true",
            description="Launch robot 1 or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_2",
            default_value="true",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_changeable", 
            default_value="false", 
            description="Use tool changeable setting?",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
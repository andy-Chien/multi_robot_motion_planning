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

#  '"0.21502 0.78661 -0.0766"'
#  [ -0.0125393, 0.0059258, 0.0176579, 0.9997479 ] xyzw
#  '"-0.58768 0.26583 -014994"'
#  [ -0.0022718, 0.0030527, 0.92198, 0.3872188 ] xyzw
# RP = {
#     'robot_1': {
#         'ur_type': 'ur5',
#         'arm_prefix': 'robot_1_',
#         'pose_xyz': '"0.18610747 0.79149527 -0.08777093"',
#         'pose_rpy': '"-0.025286 0.0114061 0.0354652"'
#     },
#     'robot_2': {
#         'ur_type': 'ur10e',
#         'arm_prefix': 'robot_2_',
#         'pose_xyz': '"0.2218704 -0.60678568 -0.1450561"',
#         'pose_rpy': '"-0.0073885 -0.001825 2.34635345"' 
#     }
# }
RP = {
    'robot_1': {
        'ur_type': 'ur5e',
        'arm_prefix': 'robot_1_',
        'pose_xyz': '"0 -0.7 0"',
        'pose_rpy': '"0 0 1.5707963"'
    },
    'robot_2': {
        'ur_type': 'ur5',
        'arm_prefix': 'robot_2_',
        'pose_xyz': '"0 0.7 0"',
        'pose_rpy': '"0 0 -1.5707963"'
    }
}


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
                PathJoinSubstitution([FindPackageShare("mr_description"), "urdf", "universal_robots", urdf_file]),
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
                "arm_prefix:=",
                RP[rn]['arm_prefix'],
                " ",
                "pose_xyz:=",
                RP[rn]['pose_xyz'],
                " ",
                "pose_rpy:=",
                RP[rn]['pose_rpy'],
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
                    [FindPackageShare("mr_config"), "srdf", "universal_robots", srdf_file]
                ),
                " ",
                "name:=ur",
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
                [FindPackageShare("mr_config"), "/launch", "/ur_moveit.launch.py"]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "ur_type": RP[rn]['ur_type'],
                "ns": rn,
                "arm_prefix": RP[rn]['arm_prefix'],
                "rviz_config_file": rn + ".rviz",
                "pose_xyz": RP[rn]['pose_xyz'],
                "pose_rpy": RP[rn]['pose_rpy'],
                "multi_arm": "true",
                "launch_rviz": "false",
                "use_fake_hardware": "false",
                "use_fake_controller": "false",
                "description_file": urdf_file,
                "srdf_file": srdf_file
            }.items(),
        )
        object_to_start.append(launch_moveit)
        

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
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_changeable", 
            default_value="false", 
            description="Use tool changeable setting?",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

# controller_manager_msgs.srv.ListControllers_Response(controller=[controller_manager_msgs.msg.ControllerState(
#     name='speed_scaling_state_broadcaster', state='active', type='ur_controllers/SpeedScalingStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['speed_scaling/speed_scaling_factor'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='io_and_status_controller', state='active', type='ur_controllers/GPIOController', claimed_interfaces=['gpio/standard_digital_output_cmd_0', 'gpio/standard_digital_output_cmd_1', 'gpio/standard_digital_output_cmd_2', 'gpio/standard_digital_output_cmd_3', 'gpio/standard_digital_output_cmd_4', 'gpio/standard_digital_output_cmd_5', 'gpio/standard_digital_output_cmd_6', 'gpio/standard_digital_output_cmd_7', 'gpio/standard_digital_output_cmd_8', 'gpio/standard_digital_output_cmd_9', 'gpio/standard_digital_output_cmd_10', 'gpio/standard_digital_output_cmd_11', 'gpio/standard_digital_output_cmd_12', 'gpio/standard_digital_output_cmd_13', 'gpio/standard_digital_output_cmd_14', 'gpio/standard_digital_output_cmd_15', 'gpio/standard_digital_output_cmd_16', 'gpio/standard_digital_output_cmd_17', 'gpio/standard_analog_output_cmd_0', 'gpio/standard_analog_output_cmd_1', 'gpio/tool_voltage_cmd', 'gpio/io_async_success', 'speed_scaling/target_speed_fraction_cmd', 'speed_scaling/target_speed_fraction_async_success', 'resend_robot_program/resend_robot_program_cmd', 'resend_robot_program/resend_robot_program_async_success', 'payload/mass', 'payload/cog.x', 'payload/cog.y', 'payload/cog.z', 'payload/payload_async_success', 'zero_ftsensor/zero_ftsensor_cmd', 'zero_ftsensor/zero_ftsensor_async_success'], required_command_interfaces=['gpio/standard_digital_output_cmd_0', 'gpio/standard_digital_output_cmd_1', 'gpio/standard_digital_output_cmd_2', 'gpio/standard_digital_output_cmd_3', 'gpio/standard_digital_output_cmd_4', 'gpio/standard_digital_output_cmd_5', 'gpio/standard_digital_output_cmd_6', 'gpio/standard_digital_output_cmd_7', 'gpio/standard_digital_output_cmd_8', 'gpio/standard_digital_output_cmd_9', 'gpio/standard_digital_output_cmd_10', 'gpio/standard_digital_output_cmd_11', 'gpio/standard_digital_output_cmd_12', 'gpio/standard_digital_output_cmd_13', 'gpio/standard_digital_output_cmd_14', 'gpio/standard_digital_output_cmd_15', 'gpio/standard_digital_output_cmd_16', 'gpio/standard_digital_output_cmd_17', 'gpio/standard_analog_output_cmd_0', 'gpio/standard_analog_output_cmd_1', 'gpio/tool_voltage_cmd', 'gpio/io_async_success', 'speed_scaling/target_speed_fraction_cmd', 'speed_scaling/target_speed_fraction_async_success', 'resend_robot_program/resend_robot_program_cmd', 'resend_robot_program/resend_robot_program_async_success', 'payload/mass', 'payload/cog.x', 'payload/cog.y', 'payload/cog.z', 'payload/payload_async_success', 'zero_ftsensor/zero_ftsensor_cmd', 'zero_ftsensor/zero_ftsensor_async_success'], required_state_interfaces=['gpio/digital_output_0', 'gpio/digital_output_1', 'gpio/digital_output_2', 'gpio/digital_output_3', 'gpio/digital_output_4', 'gpio/digital_output_5', 'gpio/digital_output_6', 'gpio/digital_output_7', 'gpio/digital_output_8', 'gpio/digital_output_9', 'gpio/digital_output_10', 'gpio/digital_output_11', 'gpio/digital_output_12', 'gpio/digital_output_13', 'gpio/digital_output_14', 'gpio/digital_output_15', 'gpio/digital_output_16', 'gpio/digital_output_17', 'gpio/digital_input_0', 'gpio/digital_input_1', 'gpio/digital_input_2', 'gpio/digital_input_3', 'gpio/digital_input_4', 'gpio/digital_input_5', 'gpio/digital_input_6', 'gpio/digital_input_7', 'gpio/digital_input_8', 'gpio/digital_input_9', 'gpio/digital_input_10', 'gpio/digital_input_11', 'gpio/digital_input_12', 'gpio/digital_input_13', 'gpio/digital_input_14', 'gpio/digital_input_15', 'gpio/digital_input_16', 'gpio/digital_input_17', 'gpio/standard_analog_output_0', 'gpio/standard_analog_output_1', 'gpio/standard_analog_input_0', 'gpio/standard_analog_input_1', 'gpio/analog_io_type_0', 'gpio/analog_io_type_1', 'gpio/analog_io_type_2', 'gpio/analog_io_type_3', 'gpio/tool_mode', 'gpio/tool_output_voltage', 'gpio/tool_output_current', 'gpio/tool_temperature', 'gpio/tool_analog_input_0', 'gpio/tool_analog_input_1', 'gpio/tool_analog_input_type_0', 'gpio/tool_analog_input_type_1', 'gpio/robot_mode', 'gpio/robot_status_bit_0', 'gpio/robot_status_bit_1', 'gpio/robot_status_bit_2', 'gpio/robot_status_bit_3', 'gpio/safety_mode', 'gpio/safety_status_bit_0', 'gpio/safety_status_bit_1', 'gpio/safety_status_bit_2', 'gpio/safety_status_bit_3', 'gpio/safety_status_bit_4', 'gpio/safety_status_bit_5', 'gpio/safety_status_bit_6', 'gpio/safety_status_bit_7', 'gpio/safety_status_bit_8', 'gpio/safety_status_bit_9', 'gpio/safety_status_bit_10', 'system_interface/initialized', 'gpio/program_running'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='mr_joint_trajectory_controller', state='active', type='mr_controllers/MRJointTrajectoryController', claimed_interfaces=['robot_2_shoulder_pan_joint/position', 'robot_2_shoulder_lift_joint/position', 'robot_2_elbow_joint/position', 'robot_2_wrist_1_joint/position', 'robot_2_wrist_2_joint/position', 'robot_2_wrist_3_joint/position'], required_command_interfaces=['robot_2_shoulder_pan_joint/position', 'robot_2_shoulder_lift_joint/position', 'robot_2_elbow_joint/position', 'robot_2_wrist_1_joint/position', 'robot_2_wrist_2_joint/position', 'robot_2_wrist_3_joint/position'], required_state_interfaces=['robot_2_shoulder_pan_joint/position', 'robot_2_shoulder_pan_joint/velocity', 'robot_2_shoulder_lift_joint/position', 'robot_2_shoulder_lift_joint/velocity', 'robot_2_elbow_joint/position', 'robot_2_elbow_joint/velocity', 'robot_2_wrist_1_joint/position', 'robot_2_wrist_1_joint/velocity', 'robot_2_wrist_2_joint/position', 'robot_2_wrist_2_joint/velocity', 'robot_2_wrist_3_joint/position', 'robot_2_wrist_3_joint/velocity'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['gpio/analog_io_type_0', 'gpio/analog_io_type_1', 'gpio/analog_io_type_2', 'gpio/analog_io_type_3', 'gpio/digital_input_0', 'gpio/digital_input_1', 'gpio/digital_input_10', 'gpio/digital_input_11', 'gpio/digital_input_12', 'gpio/digital_input_13', 'gpio/digital_input_14', 'gpio/digital_input_15', 'gpio/digital_input_16', 'gpio/digital_input_17', 'gpio/digital_input_2', 'gpio/digital_input_3', 'gpio/digital_input_4', 'gpio/digital_input_5', 'gpio/digital_input_6', 'gpio/digital_input_7', 'gpio/digital_input_8', 'gpio/digital_input_9', 'gpio/digital_output_0', 'gpio/digital_output_1', 'gpio/digital_output_10', 'gpio/digital_output_11', 'gpio/digital_output_12', 'gpio/digital_output_13', 'gpio/digital_output_14', 'gpio/digital_output_15', 'gpio/digital_output_16', 'gpio/digital_output_17', 'gpio/digital_output_2', 'gpio/digital_output_3', 'gpio/digital_output_4', 'gpio/digital_output_5', 'gpio/digital_output_6', 'gpio/digital_output_7', 'gpio/digital_output_8', 'gpio/digital_output_9', 'gpio/program_running', 'gpio/robot_mode', 'gpio/robot_status_bit_0', 'gpio/robot_status_bit_1', 'gpio/robot_status_bit_2', 'gpio/robot_status_bit_3', 'gpio/safety_mode', 'gpio/safety_status_bit_0', 'gpio/safety_status_bit_1', 'gpio/safety_status_bit_10', 'gpio/safety_status_bit_2', 'gpio/safety_status_bit_3', 'gpio/safety_status_bit_4', 'gpio/safety_status_bit_5', 'gpio/safety_status_bit_6', 'gpio/safety_status_bit_7', 'gpio/safety_status_bit_8', 'gpio/safety_status_bit_9', 'gpio/standard_analog_input_0', 'gpio/standard_analog_input_1', 'gpio/standard_analog_output_0', 'gpio/standard_analog_output_1', 'gpio/tool_analog_input_0', 'gpio/tool_analog_input_1', 'gpio/tool_analog_input_type_0', 'gpio/tool_analog_input_type_1', 'gpio/tool_mode', 'gpio/tool_output_current', 'gpio/tool_output_voltage', 'gpio/tool_temperature', 'robot_2_elbow_joint/effort', 'robot_2_elbow_joint/position', 'robot_2_elbow_joint/velocity', 'robot_2_shoulder_lift_joint/effort', 'robot_2_shoulder_lift_joint/position', 'robot_2_shoulder_lift_joint/velocity', 'robot_2_shoulder_pan_joint/effort', 'robot_2_shoulder_pan_joint/position', 'robot_2_shoulder_pan_joint/velocity', 'robot_2_wrist_1_joint/effort', 'robot_2_wrist_1_joint/position', 'robot_2_wrist_1_joint/velocity', 'robot_2_wrist_2_joint/effort', 'robot_2_wrist_2_joint/position', 'robot_2_wrist_2_joint/velocity', 'robot_2_wrist_3_joint/effort', 'robot_2_wrist_3_joint/position', 'robot_2_wrist_3_joint/velocity', 'speed_scaling/speed_scaling_factor', 'system_interface/initialized', 'tcp_fts_sensor/force.x', 'tcp_fts_sensor/force.y', 'tcp_fts_sensor/force.z', 'tcp_fts_sensor/torque.x', 'tcp_fts_sensor/torque.y', 'tcp_fts_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='forward_position_controller', state='inactive', type='position_controllers/JointGroupPositionController', claimed_interfaces=[], required_command_interfaces=['robot_2_shoulder_pan_joint/position', 'robot_2_shoulder_lift_joint/position', 'robot_2_elbow_joint/position', 'robot_2_wrist_1_joint/position', 'robot_2_wrist_2_joint/position', 'robot_2_wrist_3_joint/position'], required_state_interfaces=[], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='force_torque_sensor_broadcaster', state='active', type='force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['tcp_fts_sensor/force.x', 'tcp_fts_sensor/force.y', 'tcp_fts_sensor/force.z', 'tcp_fts_sensor/torque.x', 'tcp_fts_sensor/torque.y', 'tcp_fts_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[])])

# controller_manager_msgs.srv.ListControllers_Response(controller=[controller_manager_msgs.msg.ControllerState(
#     name='forward_position_controller', state='inactive', type='position_controllers/JointGroupPositionController', claimed_interfaces=[], required_command_interfaces=['robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_lift_joint/position', 'robot_1_elbow_joint/position', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_3_joint/position'], required_state_interfaces=[], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='mr_joint_trajectory_controller', state='inactive', type='mr_controllers/MRJointTrajectoryController', claimed_interfaces=[], required_command_interfaces=['robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_lift_joint/position', 'robot_1_elbow_joint/position', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_3_joint/position'], required_state_interfaces=['robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_pan_joint/velocity', 'robot_1_shoulder_lift_joint/position', 'robot_1_shoulder_lift_joint/velocity', 'robot_1_elbow_joint/position', 'robot_1_elbow_joint/velocity', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_1_joint/velocity', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_2_joint/velocity', 'robot_1_wrist_3_joint/position', 'robot_1_wrist_3_joint/velocity'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='force_torque_sensor_broadcaster', state='active', type='force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['tcp_fts_sensor/force.x', 'tcp_fts_sensor/force.y', 'tcp_fts_sensor/force.z', 'tcp_fts_sensor/torque.x', 'tcp_fts_sensor/torque.y', 'tcp_fts_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['gpio/analog_io_type_0', 'gpio/analog_io_type_1', 'gpio/analog_io_type_2', 'gpio/analog_io_type_3', 'gpio/digital_input_0', 'gpio/digital_input_1', 'gpio/digital_input_10', 'gpio/digital_input_11', 'gpio/digital_input_12', 'gpio/digital_input_13', 'gpio/digital_input_14', 'gpio/digital_input_15', 'gpio/digital_input_16', 'gpio/digital_input_17', 'gpio/digital_input_2', 'gpio/digital_input_3', 'gpio/digital_input_4', 'gpio/digital_input_5', 'gpio/digital_input_6', 'gpio/digital_input_7', 'gpio/digital_input_8', 'gpio/digital_input_9', 'gpio/digital_output_0', 'gpio/digital_output_1', 'gpio/digital_output_10', 'gpio/digital_output_11', 'gpio/digital_output_12', 'gpio/digital_output_13', 'gpio/digital_output_14', 'gpio/digital_output_15', 'gpio/digital_output_16', 'gpio/digital_output_17', 'gpio/digital_output_2', 'gpio/digital_output_3', 'gpio/digital_output_4', 'gpio/digital_output_5', 'gpio/digital_output_6', 'gpio/digital_output_7', 'gpio/digital_output_8', 'gpio/digital_output_9', 'gpio/program_running', 'gpio/robot_mode', 'gpio/robot_status_bit_0', 'gpio/robot_status_bit_1', 'gpio/robot_status_bit_2', 'gpio/robot_status_bit_3', 'gpio/safety_mode', 'gpio/safety_status_bit_0', 'gpio/safety_status_bit_1', 'gpio/safety_status_bit_10', 'gpio/safety_status_bit_2', 'gpio/safety_status_bit_3', 'gpio/safety_status_bit_4', 'gpio/safety_status_bit_5', 'gpio/safety_status_bit_6', 'gpio/safety_status_bit_7', 'gpio/safety_status_bit_8', 'gpio/safety_status_bit_9', 'gpio/standard_analog_input_0', 'gpio/standard_analog_input_1', 'gpio/standard_analog_output_0', 'gpio/standard_analog_output_1', 'gpio/tool_analog_input_0', 'gpio/tool_analog_input_1', 'gpio/tool_analog_input_type_0', 'gpio/tool_analog_input_type_1', 'gpio/tool_mode', 'gpio/tool_output_current', 'gpio/tool_output_voltage', 'gpio/tool_temperature', 'robot_1_elbow_joint/effort', 'robot_1_elbow_joint/position', 'robot_1_elbow_joint/velocity', 'robot_1_shoulder_lift_joint/effort', 'robot_1_shoulder_lift_joint/position', 'robot_1_shoulder_lift_joint/velocity', 'robot_1_shoulder_pan_joint/effort', 'robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_pan_joint/velocity', 'robot_1_wrist_1_joint/effort', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_1_joint/velocity', 'robot_1_wrist_2_joint/effort', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_2_joint/velocity', 'robot_1_wrist_3_joint/effort', 'robot_1_wrist_3_joint/position', 'robot_1_wrist_3_joint/velocity', 'speed_scaling/speed_scaling_factor', 'system_interface/initialized', 'tcp_fts_sensor/force.x', 'tcp_fts_sensor/force.y', 'tcp_fts_sensor/force.z', 'tcp_fts_sensor/torque.x', 'tcp_fts_sensor/torque.y', 'tcp_fts_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='speed_scaling_state_broadcaster', state='active', type='ur_controllers/SpeedScalingStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['speed_scaling/speed_scaling_factor'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[])])

# controller_manager_msgs.srv.ListControllers_Response(controller=[controller_manager_msgs.msg.ControllerState(
#     name='forward_position_controller', state='inactive', type='position_controllers/JointGroupPositionController', claimed_interfaces=[], required_command_interfaces=['robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_lift_joint/position', 'robot_1_elbow_joint/position', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_3_joint/position'], required_state_interfaces=[], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='mr_joint_trajectory_controller', state='inactive', type='mr_controllers/MRJointTrajectoryController', claimed_interfaces=[], required_command_interfaces=['robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_lift_joint/position', 'robot_1_elbow_joint/position', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_3_joint/position'], required_state_interfaces=['robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_pan_joint/velocity', 'robot_1_shoulder_lift_joint/position', 'robot_1_shoulder_lift_joint/velocity', 'robot_1_elbow_joint/position', 'robot_1_elbow_joint/velocity', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_1_joint/velocity', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_2_joint/velocity', 'robot_1_wrist_3_joint/position', 'robot_1_wrist_3_joint/velocity'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='force_torque_sensor_broadcaster', state='active', type='force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['tcp_fts_sensor/force.x', 'tcp_fts_sensor/force.y', 'tcp_fts_sensor/force.z', 'tcp_fts_sensor/torque.x', 'tcp_fts_sensor/torque.y', 'tcp_fts_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['gpio/analog_io_type_0', 'gpio/analog_io_type_1', 'gpio/analog_io_type_2', 'gpio/analog_io_type_3', 'gpio/digital_input_0', 'gpio/digital_input_1', 'gpio/digital_input_10', 'gpio/digital_input_11', 'gpio/digital_input_12', 'gpio/digital_input_13', 'gpio/digital_input_14', 'gpio/digital_input_15', 'gpio/digital_input_16', 'gpio/digital_input_17', 'gpio/digital_input_2', 'gpio/digital_input_3', 'gpio/digital_input_4', 'gpio/digital_input_5', 'gpio/digital_input_6', 'gpio/digital_input_7', 'gpio/digital_input_8', 'gpio/digital_input_9', 'gpio/digital_output_0', 'gpio/digital_output_1', 'gpio/digital_output_10', 'gpio/digital_output_11', 'gpio/digital_output_12', 'gpio/digital_output_13', 'gpio/digital_output_14', 'gpio/digital_output_15', 'gpio/digital_output_16', 'gpio/digital_output_17', 'gpio/digital_output_2', 'gpio/digital_output_3', 'gpio/digital_output_4', 'gpio/digital_output_5', 'gpio/digital_output_6', 'gpio/digital_output_7', 'gpio/digital_output_8', 'gpio/digital_output_9', 'gpio/program_running', 'gpio/robot_mode', 'gpio/robot_status_bit_0', 'gpio/robot_status_bit_1', 'gpio/robot_status_bit_2', 'gpio/robot_status_bit_3', 'gpio/safety_mode', 'gpio/safety_status_bit_0', 'gpio/safety_status_bit_1', 'gpio/safety_status_bit_10', 'gpio/safety_status_bit_2', 'gpio/safety_status_bit_3', 'gpio/safety_status_bit_4', 'gpio/safety_status_bit_5', 'gpio/safety_status_bit_6', 'gpio/safety_status_bit_7', 'gpio/safety_status_bit_8', 'gpio/safety_status_bit_9', 'gpio/standard_analog_input_0', 'gpio/standard_analog_input_1', 'gpio/standard_analog_output_0', 'gpio/standard_analog_output_1', 'gpio/tool_analog_input_0', 'gpio/tool_analog_input_1', 'gpio/tool_analog_input_type_0', 'gpio/tool_analog_input_type_1', 'gpio/tool_mode', 'gpio/tool_output_current', 'gpio/tool_output_voltage', 'gpio/tool_temperature', 'robot_1_elbow_joint/effort', 'robot_1_elbow_joint/position', 'robot_1_elbow_joint/velocity', 'robot_1_shoulder_lift_joint/effort', 'robot_1_shoulder_lift_joint/position', 'robot_1_shoulder_lift_joint/velocity', 'robot_1_shoulder_pan_joint/effort', 'robot_1_shoulder_pan_joint/position', 'robot_1_shoulder_pan_joint/velocity', 'robot_1_wrist_1_joint/effort', 'robot_1_wrist_1_joint/position', 'robot_1_wrist_1_joint/velocity', 'robot_1_wrist_2_joint/effort', 'robot_1_wrist_2_joint/position', 'robot_1_wrist_2_joint/velocity', 'robot_1_wrist_3_joint/effort', 'robot_1_wrist_3_joint/position', 'robot_1_wrist_3_joint/velocity', 'speed_scaling/speed_scaling_factor', 'system_interface/initialized', 'tcp_fts_sensor/force.x', 'tcp_fts_sensor/force.y', 'tcp_fts_sensor/force.z', 'tcp_fts_sensor/torque.x', 'tcp_fts_sensor/torque.y', 'tcp_fts_sensor/torque.z'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(
#     name='speed_scaling_state_broadcaster', state='active', type='ur_controllers/SpeedScalingStateBroadcaster', claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['speed_scaling/speed_scaling_factor'], is_chainable=False, is_chained=False, reference_interfaces=[], chain_connections=[])])

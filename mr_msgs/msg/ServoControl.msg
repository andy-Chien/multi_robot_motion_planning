uint8 STOP = 0
uint8 ARM = 1
uint8 BASE = 2
uint8 HYBRID = 3
uint8 COMBINE = 4
uint8 SINGLE_JOINT = 5

uint8 BASE_FRAME = 0
uint8 WORLD_FRAME = 1
uint8 TOOL_FRAME = 2

uint8 control_mode
uint8 arm_cmd_frame
uint8 base_cmd_frame

std_msgs/Header header
geometry_msgs/Twist arm_cmd
geometry_msgs/Twist base_cmd

# joint and value for single joint control mode
string joint
float64 value

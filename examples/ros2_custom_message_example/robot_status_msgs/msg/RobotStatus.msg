# RobotStatus.msg
# Contains information about the robot's current status

std_msgs/msg/Header header
uint8 status_code
float32 battery_level
string message
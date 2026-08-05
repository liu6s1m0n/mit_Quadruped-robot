# 状态估计消息
std_msgs/Header header

# 机身位姿：位置和姿态均在世界坐标系中
geometry_msgs/Pose body_pose

# 机身速度：
# linear：世界坐标系中的线速度
# angular：世界坐标系中的角速度
geometry_msgs/Twist body_twist

# 机身坐标系中的角速度
geometry_msgs/Vector3 body_angular_velocity

# 状态估计是否有效
bool valid

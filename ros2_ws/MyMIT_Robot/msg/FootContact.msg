# 足端接触状态消息
std_msgs/Header header

# 四条腿是否接触地面
# 顺序：FR、FL、RR、RL
bool[4] contact

# 四条腿的接触置信度，范围为 0.0～1.0
# 0.0 表示不接触，1.0 表示确定接触
float64[4] contact_probability

# 足端受到的法向力，单位：N
# 顺序：FR、FL、RR、RL
float64[4] normal_force

# 接触状态是否有效
bool valid

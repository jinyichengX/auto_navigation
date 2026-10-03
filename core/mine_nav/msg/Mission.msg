# 井下任务描述（任务层 -> 探索器）
# 坐标系：camera_init（ENU，与 /projected_map 一致）
std_msgs/Header header

# idle / explore / goto / return_home / abort
string mode

# 优先级：数字越大越优先
uint8 priority

# 探索/作业范围（轴对齐矩形）
geometry_msgs/Point area_min
geometry_msgs/Point area_max

# 人给的目标点；goto 模式必填，其它模式可忽略
geometry_msgs/Point goal

# 是否启用目标点
bool has_goal

# 禁飞区：每 2 个点一组 (min,max)，长度必须为偶数
geometry_msgs/Point[] no_fly_zones

# 建议飞行高度（ENU，向上为正，米）
float32 fly_alt_enu

# 完成后是否自动回原点
bool return_home_when_done

# 可读说明
string note

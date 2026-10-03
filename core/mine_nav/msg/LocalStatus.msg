# 局部控制器状态反馈
std_msgs/Header header

# waiting_path / tracking / avoiding / stuck / arrived / idle
string status

# 当前跟踪的全局路径点索引（从 0 开始）
uint32 path_index
uint32 path_size

# 是否正在绕障
bool avoiding

# 距当前目标点距离（米）
float32 remain_m

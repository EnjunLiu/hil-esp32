uint32 seq
int64 stamp_us

# 无人船坐标系下的期望二维位移
float32 desired_x
float32 desired_y

# 可通过IMU传感器获得的数据
float32 surge_velocity
float32 yaw_rate

bool valid
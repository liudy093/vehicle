# V2X向Planner发送的信息
# 话题名称：v2x_to_planner_data
# 话题接口：car_interfaces/msg/VtxInterface.msg

float64 timestamp       # 时间戳

# car 信息
int32 car_id
int32 platoon_id
float64 longitude
float64 latitude
float64 speed
float64 acceleration
float64 yaw

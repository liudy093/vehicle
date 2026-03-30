# 车辆控制数据话题
# 话题名称: pid_data
# 话题接口: car_interfaces/msg/PidInterface.msg

float64     timestamp               # 数据帧时间戳
float32     arc_position            # 弧长位置 m
float32     velocity                # 车辆速度m/s
float32     acceleration            # 车辆加速度m/s^2
float32     angle                   # 车辆转角，方向盘转角
uint8       gear                    # 01：驻车；02：空挡（N）；03：前进（D）；04：后退（R）；05：无效；
uint8       throttle_percentage     # 油门踏板开度（0-100%）   取值：0-100
uint8       braking_percentage      # 刹车踏板开度（0-100%） 取值： 0-100
float32     turn_signals            #转向灯 -1：左转，0：直行，1：右转
float32     process_time            # 进程处理时间

# 虚拟leader数据（仅1号车使用）
float32     vleader_arc_position    # 虚拟leader弧长位置 m
float32     vleader_velocity        # 虚拟leader速度 m/s
float32     vleader_acceleration    # 虚拟leader加速度 m/s^2

# 参考/指令值（纵向）
float32     arc_position_ref        # 参考弧长位置 m
float32     velocity_ref            # 参考速度 m/s
float32     acceleration_ref        # 参考加速度 m/s^2
uint8       throttle_cmd_percentage # 控制器输出的油门指令（0-100%）
uint8       braking_cmd_percentage  # 控制器输出的制动指令（0-100%）
uint8       gear_cmd                # 控制器输出的档位指令

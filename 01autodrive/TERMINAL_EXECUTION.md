# 终端执行文档

本文档用于 `vehicle2-AutoDrive` 在“不使用 HMI、只通过终端操作”的情况下启动自动驾驶流程。

适用场景：

- 不启动 `ros2 run hmi hmi`
- 直接在终端加载地图
- 直接在终端发布起终点
- 直接在终端启动 `tracker`

不适用场景：

- 需要图形界面选地图、选终点
- 需要在 HMI 中点击 `Plan / Replan`

## 0. 本机仓库一键脚本

如果你是在本机仓库里远程控制 `vehicle2`，优先使用下面两个脚本：

```bash
cd /Users/ldy/Code/Perception
./scripts/autodrive/start_vehicle_autodrive.sh vehicle2
./scripts/autodrive/stop_vehicle_autodrive.sh vehicle2
```

默认行为：

- 远端车辆：`nvidia@192.168.0.109`
- 地图：`visit_1.json`
- 默认终点：`end_1`
- `start` 会按就绪状态推进：等待有效 `/fusion_data` 经纬度、确认 `/global_path_planning_data` 已生成有效路径后，再启动自动驾驶
- `start` 不再依赖固定的 `sleep 5` / `sleep 2` 串行等待；实际发车时间主要取决于定位和全局路径何时就绪
- `start` 会额外挂一个到点监控；车辆进入终点附近并稳定低速后，会自动执行收尾关闭
- `stop` 用于人工中止、异常兜底或清理残留
- 默认到点距离阈值：`3.0m`
- 当前已知限制：自动收尾只会关闭 ROS 自动驾驶链路，不会额外发送“切回 P/N 挡”的底盘指令

切换终点示例：

```bash
./scripts/autodrive/start_vehicle_autodrive.sh vehicle2 end_2
./scripts/autodrive/start_vehicle_autodrive.sh vehicle2 end_3
```

脚本会提示输入 SSH 密码；如果你已经设置环境变量 `VEHICLE_PASSWORD`，则不会再次提示。

默认监控日志：

- `/home/nvidia/AutoDrive/datas/logs/remote_start/arrival_monitor.log`

当前实测结论：

- 已在 `vehicle2` 上实车验证 `visit_1.json + end_1`
- `3.0m` 距离阈值下，车辆到点并稳定低速后，监控日志会出现 `ARRIVAL_CONFIRMED -> AUTO_STOP_COMPLETED`
- 自动收尾后，自动驾驶相关进程和 ROS 节点会退出
- 车辆是否保持在 `D` 挡，不由当前脚本处理

## 1. 前提条件

默认工作目录：

```bash
cd ~/AutoDrive
```

每开一个新终端，都先执行：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

如果当前工作区还没有编译过，先在任意一个终端执行一次：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```

## 2. 推荐终端分工

建议至少开 5 个终端。

- 终端 1：基础节点
- 终端 2：激光雷达驱动
- 终端 3：激光雷达障碍物检测
- 终端 4：全局规划
- 终端 5：发布起终点并启动自动驾驶

## 3. 启动步骤

### 3.1 终端 1：启动基础节点

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_defaults.py
```

这个命令会启动：

- `car_ori`
- `sonic_obstacle`
- `regulator`
- `gps`
- `imu`
- `fusion`
- `net_work`

### 3.2 终端 2：启动激光雷达驱动

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_lidar.py
```

### 3.3 终端 3：启动激光雷达障碍物检测

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_lidar_object.py
```

### 3.4 终端 4：加载地图并启动全局规划

以 `visit_1.json` 为例：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_global_planner.py use_trajectory:=False file_path:=/home/$USER/AutoDrive/datas/maps/visit_1.json
```

说明：

- `use_trajectory:=False` 表示使用地图文件，不是轨迹文件
- `file_path` 必须指向实际存在的地图 JSON
- `visit_1.json` 是有效地图文件

## 4. 发布起终点

### 4.1 先确认车辆当前位置已经进入 `/fusion_data`

在终端 5 执行：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
timeout 10 ros2 topic echo /fusion_data car_interfaces/msg/FusionInterface --csv | head -n 1
```

如果长时间没有输出，不要继续后面的步骤，先检查：

- `launch_defaults.py` 是否正常启动
- GPS / IMU / 车辆状态是否正常
- `fusion` 是否正常发布

### 4.2 获取当前位置作为起点

推荐使用下面这段 Python 订阅脚本直接从 `/fusion_data` 读取当前经纬度：

```bash
read START_LON START_LAT < <(python3 - <<'PY'
import rclpy
from rclpy.node import Node
from car_interfaces.msg import FusionInterface

class Grabber(Node):
    def __init__(self):
        super().__init__('fusion_grabber')
        self.msg = None
        self.create_subscription(FusionInterface, '/fusion_data', self.cb, 10)

    def cb(self, msg):
        self.msg = msg

rclpy.init()
node = Grabber()
end_ns = node.get_clock().now().nanoseconds + 15_000_000_000
while rclpy.ok() and node.msg is None and node.get_clock().now().nanoseconds < end_ns:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.msg is None:
    raise SystemExit('未获取到 /fusion_data')

print(node.msg.longitude, node.msg.latitude)
node.destroy_node()
rclpy.shutdown()
PY
)

echo "$START_LON"
echo "$START_LAT"
```

### 4.3 选择终点并发布起终点

本文档默认使用 `visit_1.json` 中的 `end_1` 作为终点。  
这和在 HMI 中选择 `end_1` 的含义一致，只是这里改为手工发布坐标。

`visit_1.json` 中 3 个终点如下：

| 终点名 | 经度 | 纬度 |
| --- | --- | --- |
| `end_1` | `113.67986377` | `34.58664073` |
| `end_2` | `113.67978869` | `34.58656693` |
| `end_3` | `113.67962258` | `34.58657090` |

如果要切换终点，只需要替换下面命令中的 `endpoint` 坐标。

发布命令如下：

```bash
ros2 topic pub --once /hmi_start_end_point_data car_interfaces/msg/HmiStartEndPointInterface \
"{timestamp: 0.0, startpoint: [$START_LON, $START_LAT], endpoint: [113.67986377, 34.58664073], process_time: 0.0}"
```

注意：

- 话题名必须是 `/hmi_start_end_point_data`
- 消息类型必须是 `car_interfaces/msg/HmiStartEndPointInterface`
- `startpoint` 和 `endpoint` 的顺序都是 `[经度, 纬度]`
- 不使用 HMI 时，不要再执行 `ros2 run hmi hmi`

## 5. 确认规划结果

发布起终点后，在终端 5 执行：

```bash
timeout 10 ros2 topic echo /global_path_planning_data car_interfaces/msg/GlobalPathPlanningInterface --csv | head -n 1
```

正常情况下应看到：

- `startpoint`
- `endpoint`
- `routedata`

如果 `routedata` 为空，通常表示以下问题之一：

- 地图没有正确加载
- 起终点不在当前地图覆盖范围内
- `/fusion_data` 的当前位置异常
- 全局规划节点还没有完成订阅或重规划

必要时可以重新发布一次起终点消息。

## 6. 启动自动驾驶

确认 `/global_path_planning_data` 里已有有效 `routedata` 后，再启动跟踪控制。

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_tracker.py
```

这个命令会启动：

- `car_decision`
- `local_path_planning`
- `pid`
- `car_control`

## 7. 最小可执行流程

如果只想快速照抄，按下面顺序执行即可。

终端 1：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_defaults.py
```

终端 2：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_lidar.py
```

终端 3：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_lidar_object.py
```

终端 4：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ./launch/launch_global_planner.py use_trajectory:=False file_path:=/home/$USER/AutoDrive/datas/maps/visit_1.json
```

终端 5：

```bash
cd ~/AutoDrive
source /opt/ros/foxy/setup.bash
source install/setup.bash

read START_LON START_LAT < <(python3 - <<'PY'
import rclpy
from rclpy.node import Node
from car_interfaces.msg import FusionInterface

class Grabber(Node):
    def __init__(self):
        super().__init__('fusion_grabber')
        self.msg = None
        self.create_subscription(FusionInterface, '/fusion_data', self.cb, 10)

    def cb(self, msg):
        self.msg = msg

rclpy.init()
node = Grabber()
end_ns = node.get_clock().now().nanoseconds + 15_000_000_000
while rclpy.ok() and node.msg is None and node.get_clock().now().nanoseconds < end_ns:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.msg is None:
    raise SystemExit('未获取到 /fusion_data')

print(node.msg.longitude, node.msg.latitude)
node.destroy_node()
rclpy.shutdown()
PY
)

ros2 topic pub --once /hmi_start_end_point_data car_interfaces/msg/HmiStartEndPointInterface \
"{timestamp: 0.0, startpoint: [$START_LON, $START_LAT], endpoint: [113.67986377, 34.58664073], process_time: 0.0}"

timeout 10 ros2 topic echo /global_path_planning_data car_interfaces/msg/GlobalPathPlanningInterface --csv | head -n 1

ros2 launch ./launch/launch_tracker.py
```

## 8. 常见错误

### 8.1 不要同时使用 HMI 和手工发布起终点

如果已经执行了：

```bash
ros2 run hmi hmi
```

就不要再用终端手工往 `/hmi_start_end_point_data` 发布起终点。
因为 HMI 会持续向同一个话题发消息，手工发的 `--once` 很容易被覆盖。

### 8.2 `/fusion_data` 没有经纬度

表现：

- `START_LON` 为空
- `START_LAT` 为空

处理：

- 先检查 `launch_defaults.py` 是否正在运行
- 检查 `/fusion_data` 是否真的在发
- 检查 GPS / IMU / 车辆底层输入是否正常

### 8.3 `/global_path_planning_data` 没有 `routedata`

处理顺序：

1. 确认地图路径是否正确
2. 确认终点是否在当前地图中
3. 确认当前位置是否合理
4. 重新发布一次起终点

### 8.4 `launch_tracker.py` 启动了但车不动

优先检查：

- `/global_path_planning_data` 是否已有有效路径
- `/fusion_data` 是否持续更新
- `car_control` 所依赖的 `can1` 是否可用
- 真车底盘、雷达、GPS 等硬件是否在线

### 8.5 已到终点但没有自动关闭

优先检查：

- `/home/nvidia/AutoDrive/datas/logs/remote_start/arrival_monitor.log` 是否出现 `ARRIVAL_WINDOW` 或 `ARRIVAL_CONFIRMED`
- `/fusion_data.carspeed` 在终点附近是否已经稳定降到较低速度
- 终点附近停车位置和地图终点是否偏差过大

必要时直接执行：

```bash
cd /Users/ldy/Code/Perception
./scripts/autodrive/stop_vehicle_autodrive.sh vehicle2
```

### 8.6 自动关闭后车辆仍显示在 `D` 挡

这是当前脚本的已知限制，不表示自动收尾失败。

原因是：

- 自动收尾目前只负责关闭 `tracker`、`planner`、`fusion` 等 ROS 自动驾驶进程
- 当前主链路里的 `pid` 默认始终下发前进挡 `D`
- `stop` 不会额外发送一帧“回 `P` 挡 / `N` 挡”的底盘命令

所以常见现象是：

- 车辆已停车
- 自动驾驶节点已退出
- 底盘最后保留的档位仍然是 `D`

如果后续要解决这个问题，需要单独设计“低速确认后，发送制动 + 切挡命令，再退出控制链路”的安全收尾逻辑。

## 9. 运行结束后如何关闭

建议每次跑完后都显式关闭相关进程，再开始下一次自动驾驶。  
否则容易出现旧节点残留、重复发布、重名节点、控制链路未释放等问题。

如果你使用的是本机仓库脚本的“完整版”启动方式：

- 正常到达终点后，远端监控会自动关闭自动驾驶链路
- 如果中途要人工接管、提前中止，或者怀疑有残留，再手工执行 `./scripts/autodrive/stop_vehicle_autodrive.sh vehicle2`
- 如果车辆已到点但没有自动收尾，先查看 `arrival_monitor.log`，再执行一次 `stop`
- 如果车辆已到点且链路已自动关闭，但底盘仍然显示 `D` 挡，这属于当前已知限制

### 9.1 如果是手工开 5 个终端启动

直接在每个终端里按 `Ctrl+C` 结束对应命令即可。

建议按下面顺序关闭：

1. 先关 `launch_tracker.py`
2. 再关 `launch_global_planner.py`
3. 再关 `launch_lidar_object.py`
4. 再关 `launch_lidar.py`
5. 最后关 `launch_defaults.py`

### 9.2 如果是通过 SSH 后台启动

优先在本机仓库执行：

```bash
cd /Users/ldy/Code/Perception
./scripts/autodrive/stop_vehicle_autodrive.sh vehicle2
```

如果需要直接在车上执行，再用下面这组命令：

```bash
pkill -f 'arrival_monitor.sh'
pkill -f 'ros2 launch ./launch/launch_tracker.py'
pkill -f 'ros2 launch ./launch/launch_global_planner.py'
pkill -f 'ros2 launch ./launch/launch_lidar_object.py'
pkill -f 'ros2 launch ./launch/launch_lidar.py'
pkill -f 'ros2 launch ./launch/launch_defaults.py'
```

### 9.3 如果怀疑还有残留节点

再补一轮按节点或可执行名清理：

```bash
pkill -f arrival_monitor.sh
pkill -f car_decision
pkill -f local_path_planning
pkill -f pid
pkill -f car_control
pkill -f global_path_planning
pkill -f fusion
pkill -f gps
pkill -f imu
pkill -f rslidar_sdk_node
pkill -f lidar_obstacle
pkill -f left_lidar_obstacle
pkill -f right_lidar_obstacle
pkill -f regulator
pkill -f sonic_obstacle
pkill -f net_work
pkill -f car_ori
```

### 9.4 关闭后建议检查

执行：

```bash
ros2 node list
```

如果仍然看到本次自动驾驶相关节点还在，就继续清理一次。

## 10. 本文档对应的地图点

本文档默认示例：

- 地图：`/home/$USER/AutoDrive/datas/maps/visit_1.json`
- 默认终点：`end_1`
- 默认终点坐标：`[113.67986377, 34.58664073]`
- 可选终点：
  - `end_1` -> `[113.67986377, 34.58664073]`
  - `end_2` -> `[113.67978869, 34.58656693]`
  - `end_3` -> `[113.67962258, 34.58657090]`

如果换地图，必须同步替换：

- `file_path`
- `endpoint` 坐标

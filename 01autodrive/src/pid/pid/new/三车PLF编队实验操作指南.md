# 三车 PLF 编队实验操作指南

> 编队控制器路径: `/home/nvidia/AutoDrive/src/pid/pid/new/lmq_mpc_plf.py`

---

## 1. 编译（首次运行或修改消息定义后）

```bash
cd /home/nvidia/AutoDrive

# 编译消息包
colcon build --packages-select car_interfaces

# 重新加载环境
source install/setup.bash
```

> **注意**: 仅修改 Python 脚本无需编译，直接运行即可，如想用节点启动方式，见1*



## 1*.注册 ROS 2 节点入口

在首次使用 `ros2 run` 启动前，必须配置包结构并注册节点。

### 配置包结构（二选一）

**方案A: 使用 lmq_mpc 包（推荐，结构更清晰）**

```bash
cd /home/nvidia/AutoDrive/src/pid

# 1. 创建新包目录
mkdir -p lmq_mpc
touch lmq_mpc/__init__.py

# 2. 复制文件
cp pid/new/lmq_mpc_plf.py lmq_mpc/
cp pid/new/sim_platoon_test_plf.py lmq_mpc/
```

修改 `setup.py`，添加包声明：
```python
packages=[
    package_name,           # 'pid'
    f'{package_name}.new',  # 'pid.new' (兼容)
    'lmq_mpc',              # 新包（推荐）
],
```

添加节点入口：
```python
'lmq_mpc = lmq_mpc.lmq_mpc_plf:main',
```

**方案B: 使用 pid.new（兼容现有结构）**

```bash
cd /home/nvidia/AutoDrive/src/pid
touch pid/new/__init__.py
```

修改 `setup.py`：
```python
packages=[package_name, f'{package_name}.new'],
```

添加节点入口：
```python
'lmq_mpc = pid.new.lmq_mpc_plf:main',
```

### 编译与加载

```bash
cd /home/nvidia/AutoDrive
# 必须编译 pid 包以生成可执行链接
colcon build --packages-select pid
source install/setup.bash
```

*注意：后续仅修改代码逻辑（不改文件名或入口函数）时无需再次编译。*

## 2. 三车角色配置

| 车辆 | platoon_id | 角色 |
|------|------------|------|
| 1号车 | 1 | 领航车 |
| 2号车 | 2 | 跟随车1（跟踪1号） |
| 3号车 | 3 | 跟随车2（跟踪2号+1号） |

---

## 3. 启动流程（每辆车执行）

### 3.1 启动 HMI 界面

```bash
cd ~/AutoDrive && ros2 run hmi hmi
```

- 点击 `CONNECT` 连接车辆
- 确认 GPS 经纬度非零，状态指示灯变绿

### 3.2 设置编队 ID

在 HMI → `Other Functions` 界面:
1. 下拉选择本车编号（1/2/3）
2. 点击 `Platoon State` 确认

### 3.3 启动 V2X 通信

**新开终端:**

```bash
ros2 run v2x v2x
```

### 3.4 加载轨迹

在 HMI → `Tracker` 界面:
1. 点击 `Load File` 选择轨迹文件（三车需相同）
2. 点击 `Start Planner`
3. 选择 `Destination` 目标点
4. 点击 `Plan`

### 3.5 启动 PLF 编队控制器

**新开终端（直接python控制）:**

```bash
cd /home/nvidia/AutoDrive
python3 -u src/pid/pid/new/lmq_mpc_plf.py
```

**新开终端（启动节点控制）*:**

在终端中使用标准 ROS 2 方式运行：

```bash
ros2 run pid lmq_mpc
```

**成功标志:**

```
>>> [SUCCESS] PLF编队控制节点已成功初始化
>>> 正在等待数据...
```

---

## 4. 开始编队

### 4.1 车辆初始位置

```
[1号车] ←── 10m ──→ [2号车] ←── 10m ──→ [3号车]
```

### 4.2 启动顺序

1. **1号车**按自动驾驶按钮 → 等待 3 秒
2. **2号车**按自动驾驶按钮 → 等待 3 秒
3. **3号车**按自动驾驶按钮

### 4.3 退出编队

- **正常退出**: 踩刹车 → 按自动驾驶按钮 → 终端 `Ctrl+C`
- **紧急接管**: 踩刹车或转方向盘

---

## 5. 数据记录

数据自动保存至:
```
/home/nvidia/AutoDrive/src/pid/data/lmq_mpc_plf_car{ID}_{日期}_{时间}.csv
```

**主要字段:**

| 字段 | 含义 |
|------|------|
| gap_front | 前车净间距 (m) |
| gap_leader | 领航车净间距 (m) |
| ego_v | 本车速度 (m/s) |
| target_a | 目标加速度 (m/s²) |
| mpc_status | MPC状态 (1正常/2回退/3安全) |

---

## 6. 常见问题

| 问题 | 解决方法 |
|------|----------|
| **节点启动后一直"等待数据"** | **必须先在 HMI 中加载轨迹！** 见下方详细说明 ⭐ |
| GPS 显示 0 | 车开到空旷处，重启 HMI |
| 收不到其他车信息 | 检查 WiFi 连接，重启 v2x |
| ImportError 消息类型 | 执行 `colcon build` 后重新 source |
| 车辆不动 | 检查轨迹是否加载、挡位是否在 D 档 |

### ⚠️ 重点：控制器"等待数据"问题

**现象：** PLF 控制器启动后显示成功，但一直输出 "正在等待数据..."，车辆不动。

**根本原因：** 控制器需要轨迹数据才能运行，但**轨迹尚未加载**。

**解决方案：**

1. **必须在 HMI 中完成轨迹加载！**（步骤 3.4）
2. 确认轨迹显示在地图上
3. **然后再启动控制器**（步骤 3.5）

**验证方法：**
```bash
# 检查轨迹话题是否有数据
ros2 topic echo /local_path_planning_data --once

# 应该看到非空的 latitude, longitude, speed 数组
```

**详细诊断：** 参考 [PLF控制器启动问题诊断.md](../../../PLF控制器启动问题诊断.md)

---

## 附录: 关键参数

```python
# lmq_mpc_plf.py 中可调参数
desired_gap = 5.0      # 期望净间距 (m)
min_gap = 1.3          # 最小安全间距 (m)
alpha = 0.7            # 间距权重 (前车70%/领航车30%)
vehicle_length = 4.4   # 车长 (m)
```

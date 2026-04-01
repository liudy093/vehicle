# Vehicle Orchestration

这个仓库的目标很单一：

通过 K3s + Argo Workflows，把车端 `autodrive` 稳定地下发到指定车辆；`mapping` 和云端服务作为按需启用的支撑链路。

默认不再走“手工 SSH 串命令”的主路径，而是：

1. 在开发机上准备镜像和配置。
2. 把工作流提交到 K3s。
3. 由 Argo 按节点标签把 Pod 调度到目标 worker 或目标车辆。
4. 在车端通过 `hostPath + hostNetwork + hostPID` 方式直接运行任务。

## 当前能力

- `autodrive` 是主入口，支持 `vehicle1`、`vehicle2`、`vehicle3`
- `mapping` 是可选支撑链路，目前只覆盖 `vehicle1`、`vehicle2`
- 双车自动驾驶脚本当前仍只针对 `vehicle1 + vehicle2`
- 镜像构建与传输现在已拆分为：
  - `autodrive` 一套
  - `mapping / cloud` 一套
  - 原来的全量脚本仍保留，兼容旧用法

## 集群与车辆信息

- K3s 控制面：`172.16.2.80`
- 通用 worker：`172.16.2.81`、`172.16.2.82`、`172.16.2.83`
- 车辆默认用户：`nvidia`

当前三辆车按开机顺序固定为：

- `vehicle1 = 192.168.3.10`
- `vehicle2 = 192.168.3.6`
- `vehicle3 = 192.168.3.2`

## 最常用路径

### 1. 只下发单车 Autodrive

第一次构建或更新过 `autodrive` 相关脚本 / 配置后：

```bash
./scripts/shared/build_vehicle_autodrive_images.sh
./scripts/shared/transfer_vehicle_autodrive_images.sh
TARGET_VEHICLE=vehicle3 TARGET_ENDPOINT=end_1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

如果镜像已经在车辆节点上，最短命令就是：

```bash
TARGET_VEHICLE=vehicle3 TARGET_ENDPOINT=end_1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

说明：

- `TARGET_VEHICLE` 必传：`vehicle1`、`vehicle2`、`vehicle3`
- `TARGET_ENDPOINT` 可选，默认 `end_1`
- 目标节点不是 `Ready` 时脚本会直接失败

### 2. 需要 Mapping / Cloud 支撑链路

```bash
./scripts/shared/build_vehicle_mapping_images.sh
./scripts/shared/transfer_vehicle_mapping_images.sh
./scripts/mapping/deploy_vehicle_mapping.sh
```

说明：

- `mapping` 当前仍只会处理 `vehicle1`、`vehicle2`
- `vehicle3` 目前不在 mapping workflow 覆盖范围内

### 3. 首次接入某台车

如果你怀疑集群基础设施或节点注册还没准备好，可以直接开：

```bash
BOOTSTRAP_CLUSTER=true TARGET_VEHICLE=vehicle3 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

这个开关在单车 `autodrive` 里当前会做：

- 安装或对齐 Argo
- 注册当前 `TARGET_VEHICLE`
- 刷新 worker 标签
- 刷新车辆 inventory ConfigMap

注意：

- 它在 `autodrive` 单车部署里已支持 `vehicle3`
- `mapping` 侧脚本的 bootstrap 仍主要围绕 `vehicle1`、`vehicle2`

## 镜像脚本

### 推荐入口

只构建自动驾驶相关镜像：

```bash
./scripts/shared/build_vehicle_autodrive_images.sh
```

只传输自动驾驶相关镜像：

```bash
./scripts/shared/transfer_vehicle_autodrive_images.sh
```

只构建 mapping / cloud 相关镜像：

```bash
./scripts/shared/build_vehicle_mapping_images.sh
```

只传输 mapping / cloud 相关镜像：

```bash
./scripts/shared/transfer_vehicle_mapping_images.sh
```

### 兼容旧入口

下面两条全量脚本仍然有效：

```bash
./scripts/shared/build_vehicle_orchestration_images.sh
./scripts/shared/transfer_vehicle_orchestration_images.sh
```

但现在更推荐用上面拆开的入口，避免每次都同时处理 `autodrive` 和 `mapping`。

## 车辆清单

车辆地址的唯一来源是：

```bash
config/shared/vehicle_hosts.env
```

当前默认内容：

```bash
VEHICLE1_HOST="nvidia@192.168.3.10"
VEHICLE2_HOST="nvidia@192.168.3.6"
VEHICLE3_HOST="nvidia@192.168.3.2"
VEHICLE1_ROOT_HOST="root@192.168.3.10"
VEHICLE2_ROOT_HOST="root@192.168.3.6"
VEHICLE3_ROOT_HOST="root@192.168.3.2"
```

如果实际开机顺序发生变化，先改这份文件，再执行构建、传输和部署。

## KUBECONFIG

脚本默认使用：

```bash
export KUBECONFIG=/tmp/perception.k3s.yaml
```

如果本机还没有这个文件，可以从控制面导出：

```bash
ssh root@172.16.2.80 "cat /etc/rancher/k3s/k3s.yaml" | sed 's/127.0.0.1/172.16.2.80/' > /tmp/perception.k3s.yaml
chmod 600 /tmp/perception.k3s.yaml
```

## Autodrive Workflow

入口脚本：

```bash
./scripts/autodrive/deploy_vehicle_autodrive.sh
```

工作流定义：

```bash
deploy/autodrive/argo/vehicle-autodrive-workflow.yaml
```

运行特点：

- 只对目标车执行
- 调度条件是节点标签 `perception.vehicle.id=<target>`
- 运行目录固定挂载 `/home/nvidia/AutoDrive`
- 提交前会把 `TARGET_VEHICLE`、`TARGET_ENDPOINT`、镜像名渲染进 workflow

单车 DAG 语义：

```text
stop-${TARGET_VEHICLE}-autodrive -> run-${TARGET_VEHICLE}-autodrive
```

双车自动驾驶入口：

```bash
./scripts/autodrive/deploy_dual_vehicle_autodrive.sh
```

说明：

- 当前只针对 `vehicle1 -> end_1`
- 和 `vehicle2 -> end_2`

## Mapping Workflow

入口脚本：

```bash
./scripts/mapping/deploy_vehicle_mapping.sh
```

工作流定义：

```bash
deploy/mapping/argo/vehicle-mapping-workflow.yaml
```

实际 DAG：

```text
deploy-cloud -> verify-cloud
deploy-cloud -> restart-vehicle1 -> verify-vehicle1
deploy-cloud -> restart-vehicle2 -> verify-vehicle2
```

运行特点：

- 只会检查 `vehicle1`、`vehicle2`
- 在线车辆才会保留对应 DAG 节点
- 如果两台车都不在线，会退化成 cloud-only workflow

## 首次把车辆加入 K3s

通用入口：

```bash
VEHICLE_ID=vehicle3 VEHICLE_HOST=nvidia@192.168.3.2 ./deploy/shared/k8s/join_vehicle_k3s_agent.sh
```

如果只是给已存在的车补注册，也可以用：

```bash
VEHICLE_ID=vehicle3 bash ./deploy/shared/k8s/register_vehicle_node.sh
```

当前仓库里保留的快捷脚本：

```bash
./deploy/shared/k8s/register_vehicle1_node.sh
./deploy/shared/k8s/register_vehicle2_node.sh
```

## 常用检查命令

查看节点状态：

```bash
kubectl get nodes -L perception.role,perception.vehicle.id
```

查看 workflow：

```bash
kubectl get wf -n argo
```

查看 Argo Pod：

```bash
kubectl get pods -n argo -o wide
```

查看某次 workflow 详情：

```bash
kubectl describe wf -n argo <workflow-name>
```

查看 autodrive 主日志：

```bash
kubectl logs -n argo <run-pod-name> -c main --tail=300
```

## 已知行为

### Autodrive

- `stop` 成功不代表 `run` 一定成功
- 关键在于 `run` 阶段能否及时拿到有效 `/global_path_planning_data`
- 偶尔会先出现一次“未获取到有效 /global_path_planning_data”，随后自动恢复
- 如果重试后仍无路径数据，workflow 会以 `LAUNCH_FAILURE_CLEANUP code=1` 失败结束

成功日志通常会依次看到：

```text
FUSION_START ...
ROUTE_READY ...
TRACKER_READY endpoint=end_1 ...
ARRIVAL_MONITOR_START ...
```

### Mapping

- 目前仍按 `vehicle1`、`vehicle2` 双车设计
- 不会自动把 `vehicle3` 纳入 mapping DAG

## 排障建议

### 1. `TARGET_VEHICLE` 没传

```bash
TARGET_VEHICLE=vehicle3 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

### 2. 目标节点不是 `Ready`

```bash
kubectl get node vehicle1
kubectl get node vehicle2
kubectl get node vehicle3
```

如果节点不存在或未就绪，优先检查：

- 车端 `k3s-agent` 是否在运行
- 节点是否已经被注册到集群
- 车辆网络是否正常

### 3. Pod 没调度到目标车

优先检查：

- 节点标签 `perception.vehicle.id`
- 节点污点 `perception.role=vehicle:NoSchedule`
- 对应 runner / `argoexec` / `pause` 镜像是否已传到车辆节点

### 4. Mapping 可以跑，但 Autodrive 起不来

优先检查：

- `/home/nvidia/AutoDrive` 是否存在且内容完整
- `01autodrive` 模型权重是否齐全
- 车端 `/dev` 挂载是否正常

### 5. `run` Pod 启动但 workflow 最终失败

重点看：

```bash
kubectl logs -n argo <run-pod-name> -c main --tail=300
```

如果看到：

```text
未获取到有效 /global_path_planning_data
LAUNCH_FAILURE_CLEANUP code=1
```

优先排查：

- 车辆初始位置是否离规划线路过远
- `launch_global_planner.log` 是否持续重规划
- 地图、定位、路径输入是否正常

## 相关文件

- `01autodrive/README.md`
- `deploy/autodrive/argo/vehicle-autodrive-workflow.yaml`
- `deploy/mapping/argo/vehicle-mapping-workflow.yaml`
- `scripts/autodrive/deploy_vehicle_autodrive.sh`
- `scripts/mapping/deploy_vehicle_mapping.sh`
- `scripts/shared/build_vehicle_autodrive_images.sh`
- `scripts/shared/transfer_vehicle_autodrive_images.sh`
- `scripts/shared/build_vehicle_mapping_images.sh`
- `scripts/shared/transfer_vehicle_mapping_images.sh`

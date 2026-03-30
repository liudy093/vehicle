# Vehicle Orchestration

这个仓库的主线已经收敛为一件事：

通过 Argo Workflows 把 `autodrive` 可靠地下发到目标车辆；`mapping` 和云端感知作为可选支撑工作流，按需补齐。

当前默认路径不是手工 SSH 串脚本，而是：

1. 在 K3s 集群里提交 Argo Workflow。
2. 由 Workflow 按节点标签把 Pod 调度到通用 worker 或车辆节点。
3. 在车端通过 `hostPath + hostNetwork + hostPID` 方式直接拉起 mapping / autodrive 运行时。

## 当前主线

- `autodrive` 是主工作流，也是默认推荐入口。
- `autodrive` 负责单车定向停止旧进程并启动新进程。
- `autodrive` 必须显式指定 `TARGET_VEHICLE=vehicle1|vehicle2`。
- `mapping` 是可选支撑工作流，负责云端部署、车辆 mapping、verify。
- `mapping` 不再是下发 autodrive 的前置必经步骤，只有在需要云端/建图链路时才运行。
- 车端默认执行模式是 `pod`，SSH 只保留为准备运行目录、同步镜像和兜底排障。

如果你现在的首要目标是“把自动驾驶通过 Argo 调度 DAG 工作流到车端”，那条最短路径就是：

```bash
./scripts/shared/build_vehicle_orchestration_images.sh
./scripts/shared/transfer_vehicle_orchestration_images.sh
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

## 架构总览

```text
Developer workstation
  -> build runner images and required tarballs
  -> transfer images to cluster workers and vehicle nodes
  -> submit Argo Workflow to K3s

K3s control plane (172.16.2.80)
  -> Argo controller / argo-server
  -> workflow submission and scheduling

General workers (172.16.2.81 / .82 / .83)
  -> deploy cloud service
  -> verify cloud API

Vehicle nodes (vehicle1 / vehicle2)
  -> run mapping runner
  -> run autodrive runner
  -> mount /home/nvidia/mapping and /home/nvidia/AutoDrive
```

默认节点职责：

- 控制面：`172.16.2.80`
- 通用 worker：`172.16.2.81`、`172.16.2.82`、`172.16.2.83`
- 车辆节点：`vehicle1`、`vehicle2`
- 车辆默认用户：`nvidia`

## 两条 Workflow

### 1. Autodrive Workflow

入口脚本：

```bash
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

默认提交的工作流定义：

- `deploy/autodrive/argo/vehicle-autodrive-workflow.yaml`

源 YAML 只保留统一的 `stop-autodrive` / `run-autodrive` 模板，提交前脚本会注入目标车辆参数和镜像。目标仍然是避免无关车辆被误操作。

实际单车 DAG 语义：

```text
stop-${TARGET_VEHICLE}-autodrive -> run-${TARGET_VEHICLE}-autodrive
```

执行特点：

- 必须显式传入 `TARGET_VEHICLE`。
- 目标车辆不是 `Ready` 会直接失败。
- 任务只会被调度到带有 `perception.vehicle.id=<target>` 的车辆节点。
- 运行目录固定挂载到 `/home/nvidia/AutoDrive`。

这也是当前“通过 Argo 调度 autodrive 到车端”的正式主入口。

如果不是第一次执行，且车端已经有正式 runner 镜像，那么最短命令就是：

```bash
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

实测中，这条命令会先停止目标车上已有 autodrive 进程，再拉起新的 `run-${TARGET_VEHICLE}-autodrive` Pod。

### 2. Mapping Workflow

入口脚本：

```bash
./scripts/mapping/deploy_vehicle_mapping.sh
```

默认提交的工作流定义：

- `deploy/mapping/argo/vehicle-mapping-workflow.yaml`

实际 DAG 语义：

```text
deploy-cloud -> verify-cloud
deploy-cloud -> restart-vehicle1 -> verify-vehicle1
deploy-cloud -> restart-vehicle2 -> verify-vehicle2
```

执行特点：

- 脚本会先检查 `vehicle1`、`vehicle2` 的 `Ready` 状态。
- 只为在线车辆保留对应 DAG 节点。
- 如果两台车都不在线，会退化成 cloud-only workflow。
- 对在线车辆，会先准备 `/home/nvidia/mapping` 运行目录，再提交 Workflow。

## 仓库结构

```text
01autodrive/   车端 autodrive 代码，运行时目标目录 /home/nvidia/AutoDrive
02mapping/     车端 mapping 代码，运行时目标目录 /home/nvidia/mapping
config/shared/ 共享车辆 IP、账号和主机名映射
deploy/autodrive/ autodrive 的 Workflow 和 runner 镜像定义
deploy/mapping/ mapping 的 Workflow 定义
deploy/shared/k8s/ K3s / Argo / perception 共享清单和注册脚本
docker/autodrive/ autodrive runner 镜像定义
docker/mapping/ mapping 相关容器运行与镜像定义
docker/shared/ ssh runner 镜像定义
scripts/autodrive/ autodrive 入口脚本
scripts/mapping/ mapping 入口脚本
scripts/shared/ 构建、传输、清理和共享库
```

补充说明：

- `01autodrive` 需要额外模型权重，见 `01autodrive/README.md`。

## 运行前准备

脚本默认使用：

```bash
export KUBECONFIG=/tmp/perception.k3s.yaml
```

本机至少需要：

- `bash`
- `python3`
- `kubectl`
- `ssh`
- `scp`

以下场景还需要：

- 构建镜像：`docker`
- 首次安装 Argo：`curl`

每天开始前，先检查车辆地址文件：

- `config/shared/vehicle_hosts.env`

当前默认内容：

```bash
VEHICLE1_HOST="nvidia@192.168.3.2"
VEHICLE2_HOST="nvidia@192.168.3.6"
VEHICLE1_ROOT_HOST="root@192.168.3.2"
VEHICLE2_ROOT_HOST="root@192.168.3.6"
```

这份文件是脚本解析车辆连接地址的唯一来源。两台车 IP 如果互换，先改这里，再执行后续命令。

## 镜像职责

- `vehicle-ssh-runner:latest`
  运行 `kubectl`、`ssh`、`scp` 一类控制面操作。
- `vehicle-autodrive-runner:latest`
  负责车端 autodrive。
- `vehicle-mapping-runner:latest`
  负责车端 mapping 和 verify。

## 快速开始

### 场景 A：只把 autodrive 下发到某一台车

如果镜像已经在 cluster worker 和车辆节点上：

```bash
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

如果是刚改过 runner 镜像，或者车端还没有最新镜像：

```bash
./scripts/shared/build_vehicle_orchestration_images.sh
./scripts/shared/transfer_vehicle_orchestration_images.sh
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

切到另一台车：

```bash
TARGET_VEHICLE=vehicle2 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

### 场景 B：需要云端和 mapping 时，再补支撑工作流

```bash
./scripts/shared/build_vehicle_orchestration_images.sh
./scripts/shared/transfer_vehicle_orchestration_images.sh
./scripts/mapping/deploy_vehicle_mapping.sh
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

### 场景 C：首次接入，先补基础设施，再下发 autodrive

```bash
BOOTSTRAP_CLUSTER=true TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

如果首次接入同时还需要云端和 mapping，再额外运行：

```bash
BOOTSTRAP_CLUSTER=true ./scripts/mapping/deploy_vehicle_mapping.sh
```

`BOOTSTRAP_CLUSTER=true` 会触发：

- 安装或对齐 Argo
- 注册 `vehicle1`、`vehicle2`
- 刷新 worker 标签和 inventory ConfigMap

## 标准操作顺序

建议按下面的顺序执行：

1. 更新 `config/shared/vehicle_hosts.env`。
2. 确认 `/tmp/perception.k3s.yaml` 可用。
3. 运行 `./scripts/shared/build_vehicle_orchestration_images.sh`。
4. 运行 `./scripts/shared/transfer_vehicle_orchestration_images.sh`。
5. 对目标车辆运行 `TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh`。
6. 如需云端和 mapping，再运行 `./scripts/mapping/deploy_vehicle_mapping.sh`。
7. 观察 Workflow、Pod、车辆节点状态。
8. 回收 mapping / cloud 支撑环境时运行 `./scripts/shared/cleanup_vehicle_orchestration.sh`。

## 执行一次 Autodrive

如果这不是第一次执行，并且正式镜像 `vehicle-autodrive-runner:latest` 已经在节点上，按下面顺序操作：

1. 确认车辆节点在线。
2. 提交单车 workflow。
3. 观察 `stop -> run` 两个阶段的日志。

执行命令：

```bash
export KUBECONFIG=/tmp/perception.k3s.yaml
kubectl get nodes -L perception.role,perception.vehicle.id
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

观察命令：

```bash
kubectl get wf -n argo
kubectl get pods -n argo -o wide
kubectl describe wf -n argo <workflow-name>
kubectl logs -n argo <run-pod-name> -c main --tail=300
```

成功时，`run` Pod 的主日志通常会依次出现：

```text
FUSION_START ...
ROUTE_READY ...
TRACKER_READY endpoint=end_1 ...
ARRIVAL_MONITOR_START ...
```

看到 `TRACKER_READY` 和 `ARRIVAL_MONITOR_START`，就说明 autodrive 已经成功拉起，进入到达监控阶段。

## Autodrive 运行现象

这条 workflow 的实测行为有几个需要提前知道：

- `stop` 阶段成功，不代表 `run` 一定成功；真正关键的是 `run` 阶段能否读到有效 `/global_path_planning_data`。
- 偶尔会先出现一次 `未获取到有效 /global_path_planning_data`，随后自动重试并恢复；这属于正常现象。
- 如果重试后仍然没有路径数据，workflow 会以 `LAUNCH_FAILURE_CLEANUP code=1` 失败结束。
- 车辆初始位置离规划线路较远时，重新规划结果会影响这一步是否成功；调整位置后重跑，可能从失败变为成功。
- 正式镜像 `vehicle-autodrive-runner:latest` 已经验证可用；当前不再使用任何 `minimal` / 测试镜像。

## 常用命令

查看工作流：

```bash
kubectl get wf -n argo
```

查看 perception 命名空间 Pod：

```bash
kubectl get pods -n perception -o wide
```

查看车辆节点状态：

```bash
kubectl get nodes -L perception.role,perception.vehicle.id
```

查看某次 Workflow 详情：

```bash
kubectl describe wf -n argo <workflow-name>
```

## 首次接入和补环境

只注册车辆节点：

```bash
./deploy/shared/k8s/register_vehicle1_node.sh
./deploy/shared/k8s/register_vehicle2_node.sh
```

只刷新车辆清单：

```bash
./deploy/shared/k8s/apply_vehicle_inventory_configmap.sh
```

只安装或对齐 Argo：

```bash
./deploy/shared/k8s/argo-install.sh
```

只启动云端：

```bash
./scripts/mapping/start_k3s_master_cloud_stack.sh
```

只下发 autodrive：

```bash
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

只清理 mapping / cloud 支撑侧，不停止 autodrive：

```bash
./scripts/shared/cleanup_vehicle_orchestration.sh
```

## 排障建议

### 1. `TARGET_VEHICLE` 没传

`deploy_vehicle_autodrive.sh` 会直接退出。传入：

```bash
TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

### 2. 车辆节点不是 `Ready`

先检查：

```bash
kubectl get node vehicle1
kubectl get node vehicle2
```

如果节点不存在或未就绪，先补注册或检查车端 k3s agent。

### 3. workflow 提交成功，但 Pod 没调度到车端

优先检查：

- 节点标签 `perception.vehicle.id`
- 节点污点 `perception.role=vehicle:NoSchedule`
- 车辆节点是否已经导入 runner / `argoexec` / `pause` 镜像

### 4. mapping 可以跑，autodrive 不起

优先检查：

- `/home/nvidia/AutoDrive` 是否存在且内容完整
- `01autodrive` 依赖的模型权重是否已补齐
- 车端 `/dev` 挂载是否正常

### 5. `run` Pod 起了，但 workflow 最终失败

优先看 `run` Pod 主日志：

```bash
kubectl logs -n argo <run-pod-name> -c main --tail=300
```

如果看到：

```text
未获取到有效 /global_path_planning_data
LAUNCH_FAILURE_CLEANUP code=1
```

说明基础 launch 已经起来了，但没有及时拿到路径规划数据。此时优先检查：

- 车辆初始位置是否偏离规划线路过远
- `launch_global_planner.log` 是否打印 `Replan !!!`
- 车端 `visit_1.json`、地图和定位输入是否正常
- 直接重跑一次是否恢复

## 相关文档

- `01autodrive/README.md`
- `deploy/autodrive/argo/vehicle-autodrive-workflow.yaml`
- `deploy/mapping/argo/vehicle-mapping-workflow.yaml`
- `scripts/autodrive/deploy_vehicle_autodrive.sh`
- `scripts/mapping/deploy_vehicle_mapping.sh`

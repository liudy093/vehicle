# Vehicle Orchestration

这个仓库用于把车端 `autodrive` 和可选的 `mapping` 支撑链路，通过 `K3s + Argo Workflows` 下发到指定节点运行。

当前的运行模型已经固定为：

- 所有 workflow 都提交到 `argo` namespace
- `autodrive` 直接调度到目标车辆节点
- `mapping-cloud` 也部署在 `argo` namespace，由通用 worker 承载
- 节点调度契约统一使用：
  - `vehicle.role=general|vehicle`
  - `vehicle.id=<vehicleX>`

## 当前能力

- `autodrive` 支持 `vehicle1`、`vehicle2`、`vehicle3`
- `mapping` 当前只覆盖 `vehicle1`、`vehicle2`
- 单车 `autodrive` 是主入口
- 双车 / 三车 `autodrive` 入口仍保留，但要求对应车辆节点在线
- 镜像入口已经拆分为两套：
  - `autodrive`
  - `mapping / cloud`

## 集群与车辆

- K3s control plane: `172.16.2.80`
- 通用 worker:
  - `172.16.2.81`
  - `172.16.2.82`
  - `172.16.2.83`
- 车辆默认用户: `nvidia`

默认车辆映射以 [`config/shared/vehicle_hosts.env`](/Users/ldy/Code/vehicle/config/shared/vehicle_hosts.env) 为准。当前默认值是：

```bash
VEHICLE1_HOST="nvidia@192.168.3.10"
VEHICLE2_HOST="nvidia@192.168.3.6"
VEHICLE3_HOST="nvidia@192.168.3.2"
VEHICLE1_ROOT_HOST="root@192.168.3.10"
VEHICLE2_ROOT_HOST="root@192.168.3.6"
VEHICLE3_ROOT_HOST="root@192.168.3.2"
```

如果车辆开机顺序变化，先更新这份文件，再执行构建、传输和部署。

## Kubeconfig

所有脚本默认使用：

```bash
export KUBECONFIG=/tmp/vehicle.k3s.yaml
```

如果本机还没有这个文件，可以从控制面导出：

```bash
ssh root@172.16.2.80 "cat /etc/rancher/k3s/k3s.yaml" \
  | sed 's/127.0.0.1/172.16.2.80/' \
  > /tmp/vehicle.k3s.yaml
chmod 600 /tmp/vehicle.k3s.yaml
```

## 最短路径

### 1. 单车 Autodrive

第一次构建或更新过 `autodrive` 相关内容后：

```bash
./scripts/shared/build_vehicle_autodrive_images.sh
./scripts/shared/transfer_vehicle_autodrive_images.sh
TARGET_VEHICLE=vehicle1 TARGET_ENDPOINT=end_1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

如果镜像已经在目标车节点上，可直接提交 workflow：

```bash
TARGET_VEHICLE=vehicle1 TARGET_ENDPOINT=end_1 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

约束：

- `TARGET_VEHICLE` 必填：`vehicle1` / `vehicle2` / `vehicle3`
- `TARGET_ENDPOINT` 可选，默认 `end_1`
- 目标节点不是 `Ready` 时脚本会直接失败

### 2. Mapping / Cloud

```bash
./scripts/shared/build_vehicle_mapping_images.sh
./scripts/shared/transfer_vehicle_mapping_images.sh
./scripts/mapping/deploy_vehicle_mapping.sh
```

约束：

- `mapping` 只处理 `vehicle1`、`vehicle2`
- 在线车辆才会保留对应 DAG 节点
- 如果两台车都不在线，会退化成 cloud-only workflow

### 3. 首次接入车辆

如果节点还没注册，可以直接用单车 `autodrive` 入口带 bootstrap：

```bash
BOOTSTRAP_CLUSTER=true TARGET_VEHICLE=vehicle3 ./scripts/autodrive/deploy_vehicle_autodrive.sh
```

这个开关当前会做：

- 安装或对齐 Argo
- 注册当前 `TARGET_VEHICLE`
- 刷新 worker 标签
- 刷新车辆 inventory ConfigMap

也可以直接走节点接入脚本：

```bash
VEHICLE_ID=vehicle3 VEHICLE_HOST=nvidia@192.168.3.2 ./deploy/shared/k8s/join_vehicle_k3s_agent.sh
```

如果只是给已存在车辆补注册：

```bash
VEHICLE_ID=vehicle3 bash ./deploy/shared/k8s/register_vehicle_node.sh
```

## 镜像入口

只构建 `autodrive` 相关镜像：

```bash
./scripts/shared/build_vehicle_autodrive_images.sh
```

只传输 `autodrive` 相关镜像：

```bash
./scripts/shared/transfer_vehicle_autodrive_images.sh
```

只构建 `mapping / cloud` 相关镜像：

```bash
./scripts/shared/build_vehicle_mapping_images.sh
```

只传输 `mapping / cloud` 相关镜像：

```bash
./scripts/shared/transfer_vehicle_mapping_images.sh
```

## Workflow 说明

### Autodrive

入口脚本：

```bash
./scripts/autodrive/deploy_vehicle_autodrive.sh
```

工作流定义：

```bash
deploy/autodrive/argo/vehicle-autodrive-workflow.yaml
```

特点：

- workflow 运行在 `argo` namespace
- 只调度到目标车
- 调度条件是节点标签 `vehicle.id=<target>`
- 节点需容忍 `vehicle.role=vehicle:NoSchedule`
- 运行目录固定挂载 `/home/nvidia/AutoDrive`

单车 DAG 语义：

```text
stop-${TARGET_VEHICLE}-autodrive -> run-${TARGET_VEHICLE}-autodrive
```

双车入口：

```bash
./scripts/autodrive/deploy_dual_vehicle_autodrive.sh
```

三车入口：

```bash
./scripts/autodrive/deploy_triple_vehicle_autodrive.sh
```

### Mapping

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

特点：

- workflow 和 `mapping-cloud` 都部署在 `argo` namespace
- `mapping-cloud` 调度到 `vehicle.role=general` 的 worker
- 车辆侧任务调度到 `vehicle.id=vehicle1|vehicle2`
- 车辆侧任务容忍 `vehicle.role=vehicle:NoSchedule`

## 常用检查命令

查看节点标签：

```bash
kubectl get nodes -L vehicle.role,vehicle.id
```

查看节点 taint：

```bash
kubectl get node vehicle1 -o jsonpath='{range .spec.taints[*]}{.key}={.value}:{.effect}{"\n"}{end}'
```

查看 workflow：

```bash
kubectl get wf -n argo
```

查看 Pod：

```bash
kubectl get pods -n argo -o wide
```

查看 `mapping-cloud` 资源：

```bash
kubectl get deploy,svc,configmap -n argo | grep mapping-cloud
```

查看某次 workflow 详情：

```bash
kubectl describe wf -n argo <workflow-name>
```

查看 `autodrive` 主日志：

```bash
kubectl logs -n argo <run-pod-name> -c main --tail=300
```

查看 `mapping-cloud` 日志：

```bash
kubectl logs -n argo deployment/mapping-cloud --tail=200
```

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

优先检查：

- 车端 `k3s-agent` 是否在运行
- 节点是否已经注册到集群
- 车辆网络是否正常

### 3. Pod 没调度到目标车

优先检查：

- 节点标签 `vehicle.id`
- 节点污点 `vehicle.role=vehicle:NoSchedule`
- 对应 runner / `argoexec` / `pause` 镜像是否已经传到车辆节点

### 4. Mapping cloud 没起来

优先检查：

- `mapping-cloud` 的 deployment / service / configmap 是否都在 `argo`
- worker 节点是否带有 `vehicle.role=general`
- `vehicle-cloud:amd64` 镜像是否已经导入 worker

### 5. Mapping 可以跑，但 Autodrive 起不来

优先检查：

- `/home/nvidia/AutoDrive` 是否存在且内容完整
- `01autodrive` 模型权重是否齐全
- 车端 `/dev` 挂载是否正常

### 6. `run` Pod 启动但 workflow 最终失败

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

- [01autodrive/README.md](/Users/ldy/Code/vehicle/01autodrive/README.md)
- [vehicle-autodrive-workflow.yaml](/Users/ldy/Code/vehicle/deploy/autodrive/argo/vehicle-autodrive-workflow.yaml)
- [vehicle-mapping-workflow.yaml](/Users/ldy/Code/vehicle/deploy/mapping/argo/vehicle-mapping-workflow.yaml)
- [mapping-cloud-configmap.yaml](/Users/ldy/Code/vehicle/deploy/shared/k8s/mapping-cloud-configmap.yaml)
- [mapping-cloud-deployment.yaml](/Users/ldy/Code/vehicle/deploy/shared/k8s/mapping-cloud-deployment.yaml)
- [mapping-cloud-service.yaml](/Users/ldy/Code/vehicle/deploy/shared/k8s/mapping-cloud-service.yaml)
- [join_vehicle_k3s_agent.sh](/Users/ldy/Code/vehicle/deploy/shared/k8s/join_vehicle_k3s_agent.sh)
- [argo-install.sh](/Users/ldy/Code/vehicle/deploy/shared/k8s/argo-install.sh)
- [deploy_vehicle_autodrive.sh](/Users/ldy/Code/vehicle/scripts/autodrive/deploy_vehicle_autodrive.sh)
- [deploy_vehicle_mapping.sh](/Users/ldy/Code/vehicle/scripts/mapping/deploy_vehicle_mapping.sh)
- [build_vehicle_autodrive_images.sh](/Users/ldy/Code/vehicle/scripts/shared/build_vehicle_autodrive_images.sh)
- [transfer_vehicle_autodrive_images.sh](/Users/ldy/Code/vehicle/scripts/shared/transfer_vehicle_autodrive_images.sh)
- [build_vehicle_mapping_images.sh](/Users/ldy/Code/vehicle/scripts/shared/build_vehicle_mapping_images.sh)
- [transfer_vehicle_mapping_images.sh](/Users/ldy/Code/vehicle/scripts/shared/transfer_vehicle_mapping_images.sh)
- [cleanup_vehicle_orchestration.sh](/Users/ldy/Code/vehicle/scripts/shared/cleanup_vehicle_orchestration.sh)

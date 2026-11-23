# PadBot E2 (派宝 E2) ROS 2 Humble 真机控制与建图导航工程

本项目面向 **广州映博智能科技有限公司 PadBot E2（派宝 E2）模块化移动机器人平台**，提供基于 **Ubuntu 22.04 + ROS 2 Humble** 的上位机通信桥接、实地激光建图与地图导出、单点/多点自主导航及动态避障验证全套工程实现。

> [!IMPORTANT]
> **真机复现指引**：拿到派宝 E2 实车后，请直接参阅核心实操手册：  
> 👉 **[派宝 E2 真机复现与建图导航实操手册](experiments/lab02_sim_slam_nav/README.md)**  
> 原厂精简文本规范请参阅仓库根目录文档：  
> 👉 **[`e2_ver_1_0_1.md`](e2_ver_1_0_1.md)**

---

## 一、核心技术路线

- **现代跨平台架构**：原厂选配工控机默认采用已停止维护的 Ubuntu 18.04 + ROS 1 Melodic；本方案采用外部 **Ubuntu 22.04 + ROS 2 Humble** 上位机，通过标准网口以 **HTTP RESTful API（`/robot/api/v1.0/...`）**解耦对接 E2 内部 Linux 导航底盘。
- **真机时间戳规范**：真机模式全局统一使用宿主机墙上时钟（**`use_sim_time: False`**）。
- **底盘核心职责划分**：激光 SLAM 栅格建图、全局重定位与局部多传感器避障由 E2 底盘内置导航系统闭环完成；ROS 2 上位机负责标准生态对接（发布 `/e2/odom`、`/e2/map`、`/e2/plan`、`/e2/action_status`，广播 TF 树，集成 RViz2 可视化监控与工程级软限速防撞保护）。

---

## 二、仓库目录结构

```text
Auto_Driving/
├── e2_ver_1_0_1.md                 ← 派宝 E2 原厂手册精简文本规范（硬件规格/接口/安全要求）
├── README.md                       ← 本文件（项目主定位与快速索引）
└── experiments/
    └── lab02_sim_slam_nav/         ← 派宝 E2 独立 colcon 工作空间
        ├── README.md               # 派宝 E2 真机复现 SOP 实操手册（6 章详尽指南）
        ├── e2_maps/                # 真机实景建图与一键导出目录（仅 .gitkeep 入库）
        │   └── .gitkeep
        └── src/
            └── yunyan_e2_bridge/   # 核心 ROS 2 桥接包（HTTP API 桥接、探针、巡航、Mock 及 RViz2）
                ├── config/         # 桥接参数与航点配置
                ├── launch/         # 真机与 Mock 启动脚本
                ├── rviz/           # 1:1 三维监控预置视图
                ├── urdf/           # E2 1:1 物理尺寸模型
                └── yunyan_e2_bridge/ # 核心节点（client、bridge、probe、patrol、mock）
```

> `build/`、`install/`、`log/` 为本地编译产物，`e2_maps/*.map|pgm|yaml` 为本地地图导出产物，均已由 [`.gitignore`](.gitignore) 自动忽略。

---

## 三、环境要求与单包构建

### 1. 系统与依赖安装（Ubuntu 22.04）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-teleop-twist-keyboard \
  python3-requests \
  python3-yaml \
  python3-pil \
  python3-flake8 \
  python3-pytest
```

### 2. 独立单包编译

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
source /opt/ros/humble/setup.bash
colcon build --packages-select yunyan_e2_bridge --symlink-install
source install/setup.bash
```

---

## 四、快速启动参考

### 1. 真机在线联调（需接实车网口并获知 `<E2_IP>:<Port>`）

```bash
# 步骤 1：零移动只读接口诊断
ros2 run yunyan_e2_bridge e2_probe --url http://<E2_IP>:<Port>

# 步骤 2：启动桥接节点与 RViz2 监控
ros2 launch yunyan_e2_bridge e2_bringup.launch.py mock:=false base_url:=http://<E2_IP>:<Port>

# 步骤 3：启动标准键盘慢速遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
```

### 2. 离线无真机 Mock 冒烟自检

```bash
# 启动内置 Mock HTTP Server + 动力学模拟 + RViz2
ros2 launch yunyan_e2_bridge e2_bringup.launch.py mock:=true
```

详细的实操分步 SOP、安全红线 Checklist 与故障排查见 **[实验 02 详细手册](experiments/lab02_sim_slam_nav/README.md)**。

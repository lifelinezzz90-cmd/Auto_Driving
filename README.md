# Auto_Driving

ROS 2 + ESP32 / PadBot E2 的移动机器人学习实验集。每个实验自包含：ROS 2 上位机包、固件/桥接代码、以及该实验的说明文档都放在 `experiments/<实验名>/` 下。

## 实验列表

| 实验 | 内容 | 阶段进度 | 状态 |
|---|---|---|---|
| [`lab01_http_car`](experiments/lab01_http_car/) | `cmd_vel` → HTTP 桥接 → 云衍小车；含键盘遥操作与 ESP32 固件 | 完成 | ✅ |
| [`lab02_sim_slam_nav`](experiments/lab02_sim_slam_nav/) | Gazebo 仿真、SLAM 建图、Nav2/APF 导航避障，以及派宝 E2 真机 HTTP 桥接与复现 SOP | **M1~M4.2 全部完成**（M4.2 = 真机复现手册 + Mock 产物清理） | ✅ |

> 派宝 E2 真机部署请直接阅读 **[实验 02 README：真机复现手册](experiments/lab02_sim_slam_nav/README.md)**；原厂手册精简文本见 [`e2_ver_1_0_1.md`](e2_ver_1_0_1.md)（仓库根目录，不放在 `src/` 下，原版 PDF 不入库）。

## 目录结构

```text
Auto_Driving/
├── e2_ver_1_0_1.md                 ← 派宝 E2 原厂手册精简文本规范
├── README.md                       ← 本文件
└── experiments/
    ├── lab01_http_car/             ← 一个实验 = 一个独立 colcon 工作空间
    │   ├── README.md
    │   ├── src/
    │   │   ├── control_learn/      # cmd_vel 订阅观察节点
    │   │   └── yunyan_bringup/     # HTTP 桥接节点 + 键盘遥操作 + launch
    │   └── firmware/
    │       ├── README.md
    │       └── yunyan_car/         # ESP32 固件（Arduino）
    └── lab02_sim_slam_nav/         ← 仿真 + SLAM + Nav2 + E2 真机桥接
        ├── README.md               # 派宝 E2 真机复现手册（6 章 SOP）；手册文本见仓库根目录
        ├── e2_maps/                # 真机导出地图目录，仅 .gitkeep 入库
        │   └── .gitkeep
        ├── report_assets/          # 报告配图与素材
        ├── scripts/                # 可视化/流程辅助脚本
        └── src/
            ├── yunyan_bringup/     # 键盘遥操作 /cmd_vel
            ├── yunyan_sim/         # Gazebo 世界、URDF、仿真 launch
            ├── yunyan_nav/         # SLAM Toolbox / Nav2 / APF 避障
            └── yunyan_e2_bridge/   # E2 RESTful API 桥接、探针、巡航、Mock
```

`build/` `install/` `log/` 都是 colcon 产物，按实验各自生成，不入库；`e2_maps/*.map|pgm|yaml` 与 `/tmp/` 导出产物同样被忽略。

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble（`source /opt/ros/humble/setup.bash`）
- Python 3.10
- 实验 02 额外依赖：`ros-humble-slam-toolbox`、`ros-humble-navigation2`、`ros-humble-nav2-bringup`、`ros-humble-xacro`、`ros-humble-rviz2`、`python3-requests`、`python3-yaml`
- Arduino IDE + ESP32 board package + [U8g2](https://github.com/olikraus/u8g2) 库（仅 lab01 烧录固件需要）

## 快速开始

每个实验都是独立工作空间，**先进到实验目录再构建**：

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav   # 或 lab01_http_car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

之后按对应实验 README 的步骤运行。真机模式必须使用 `mock:=false` 并显式传入 `base_url:=http://<E2_IP>:<Port>`，且全程 `use_sim_time: False`。

## 约定

- **每个实验独立工作空间**：互不干扰，换实验只换目录，不用清理旧 build。
- **凭据不入库**：Wi-Fi 密码等写进被 `.gitignore` 排除的本地文件，仓库只保留 `.example` 模板。
- **大体积手册 PDF 不入库**：仓库只保留根目录 `e2_ver_1_0_1.md` 文本规范。
- **提交信息遵循 [Conventional Commits](https://www.conventionalcommits.org/)**：`feat:` / `fix:` / `docs:` / `chore:` …
- 分支：`main` 上只保留可运行状态，改动走 `feature/<名字>` 分支再合并。

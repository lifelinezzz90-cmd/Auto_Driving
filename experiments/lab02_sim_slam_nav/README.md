# 实验 02：Gazebo 仿真、SLAM 建图与自主导航 (lab02_sim_slam_nav)

- **阶段 M1**：Gazebo 差速小车与激光雷达仿真环境搭建（✅ 已完成）
- **阶段 M2**：SLAM Toolbox 在线异步建图与地图保存（✅ 已完成）
- **阶段 M3**：APF 人工势场避障与 Nav2 完整导航栈集成（✅ 已完成）

## 目录结构

```
experiments/lab02_sim_slam_nav/
├── README.md                      ← 本文件
└── src/
    ├── yunyan_bringup/            # 键盘控制包 (发布 /cmd_vel)
    │   └── yunyan_bringup/
    │       └── car_teleop.py
    ├── yunyan_sim/                # 仿真核心包
    │   ├── launch/
    │   │   └── sim_bringup.launch.py  # 一键启动 Gazebo + RViz2 + 机器人
    │   ├── rviz/
    │   │   └── sim_view.rviz          # 仿真视角 (Fixed Frame: odom)
    │   ├── urdf/
    │   │   └── yunyan_car.urdf.xacro  # 包含差速与雷达插件的机器人模型
    │   └── worlds/
    │       └── slam_room.world        # 8m x 8m 非对称封闭测试房间
    └── yunyan_nav/                # 建图与导航包
        ├── config/
        │   ├── mapper_params_online_async.yaml # SLAM Toolbox 异步建图参数
        │   └── nav2_params.yaml       # Nav2 导航核心参数 (AMCL/DWB/Costmaps)
        ├── launch/
        │   ├── slam_sim.launch.py     # 一键建图脚本 (SLAM Toolbox + RViz2)
        │   └── nav2_sim.launch.py     # 一键导航脚本 (支持静态地图导航与 SLAM 联调)
        ├── maps/                      # 地图存储目录
        │   ├── slam_room_map.pgm      # 8m x 8m 封闭房间栅格图
        │   └── slam_room_map.yaml     # 地图元数据配置
        ├── rviz/
        │   ├── slam_view.rviz         # 建图视角 (Fixed Frame: map，含 /map 图层)
        │   └── nav2_view.rviz         # 导航视角 (含代价地图、规划路径、粒子群)
        └── yunyan_nav/
            ├── __init__.py
            └── apf_avoidance.py       # [方案 B] 人工势场反应式避障节点
```

## 构建与运行

在工作空间根目录下执行构建：

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 阶段 M1：基础仿真测试
1. 启动仿真与可视化：
   ```bash
   ros2 launch yunyan_sim sim_bringup.launch.py
   ```
2. 启动键盘遥操作：
   ```bash
   ros2 run yunyan_bringup car_teleop
   ```

---

### 阶段 M2：SLAM Toolbox 在线建图与存图
1. **终端 1**：启动仿真小车与封闭世界
   ```bash
   ros2 launch yunyan_sim sim_bringup.launch.py
   ```
2. **终端 2**：启动 SLAM 在线建图
   ```bash
   ros2 launch yunyan_nav slam_sim.launch.py
   ```
3. **终端 3**：启动键盘遥操作巡游建图
   ```bash
   ros2 run yunyan_bringup car_teleop
   ```
4. **终端 4**：保存建好的地图至 `maps/` 目录
   ```bash
   ros2 run nav2_map_server map_saver_cli -f src/yunyan_nav/maps/slam_room_map
   ```

---

### 阶段 M3：避障与自主导航（双方案对比）

#### 方案 B：APF 人工势场轻量级反应式避障
适用于无地图环境下的自主避障巡游与死锁脱困。
1. **终端 1**：启动仿真环境
   ```bash
   ros2 launch yunyan_sim sim_bringup.launch.py
   ```
2. **终端 2**：启动 APF 避障节点
   ```bash
   cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 run yunyan_nav apf_avoidance
   ```
   > 小车将自主前行（0.18 m/s），遇障碍物通过横向斥力平滑避让，距离障碍物小于 0.35m 时自动原地旋转脱困。

#### 方案 A（模式 1）：Nav2 已知静态地图自主导航
适用于已知环境下的点对点全局与局部动态规划导航。
1. **终端 1**：启动仿真环境
   ```bash
   ros2 launch yunyan_sim sim_bringup.launch.py
   ```
2. **终端 2**：启动 Nav2 导航栈（加载保存好的 `slam_room_map.yaml`）
   ```bash
   cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch yunyan_nav nav2_sim.launch.py
   ```
   > 节点已自动对齐出生位姿 (0,0,0)。在打开的 RViz 窗口顶部点击 **2D Goal Pose** 工具，在地图任意可行走区域给定目标点与航向角，小车即可自主寻优到达！

#### 方案 A（模式 2）：实时 SLAM 建图 + Nav2 导航联调
适用于未知环境下的探索与实时导航联调。
1. **终端 1**：启动仿真环境
   ```bash
   ros2 launch yunyan_sim sim_bringup.launch.py
   ```
2. **终端 2**：启动 SLAM + Nav2 联调
   ```bash
   cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch yunyan_nav nav2_sim.launch.py slam:=True
   ```
   > 小车将一边通过 SLAM Toolbox 异步在线拓展地图，一边接收 `2D Goal Pose` 进行实时路径规划导航。

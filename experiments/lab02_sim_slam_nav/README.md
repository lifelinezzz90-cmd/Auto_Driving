# 实验 02：派宝 E2 真机复现手册（Gazebo / SLAM / Nav2 / HTTP 桥接）

> **主线**：拿到派宝 E2（PadBot E2）真机后，**严格对照原厂手册与本套 ROS 2 Humble 工程**，按现场实车顺序完成「安全点检 → 接口探针 → 桥接上线 → 实景建图 → 辅助定位 → 安全采集航点 → 多点巡航与动态避障」。
>
> - 阶段 M1：Gazebo 差速小车与激光雷达仿真环境（✅ 已完成）
> - 阶段 M2：SLAM Toolbox 在线异步建图与存图（✅ 已完成）
> - 阶段 M3：APF 人工势场避障 + Nav2 完整导航栈（✅ 已完成）
> - 阶段 M4.1：派宝 E2 HTTP RESTful API ROS 2 桥接包（✅ 已完成）
> - **阶段 M4.2：真机复现 SOP、Mock 产物清理与部署文档（✅ 本手册）**
>
> 原厂手册精简文本规范见仓库根目录 [`e2_ver_1_0_1.md`](../../e2_ver_1_0_1.md)（**不放在 `src/` 下**）；大体积原版 PDF 手册按 `.gitignore` 规则不入仓库。

---

## 1. 项目总览与技术架构

本工作空间 `experiments/lab02_sim_slam_nav/` 是一个独立的 `colcon` 工作空间，包含 4 个核心 ROS 2 功能包：

| 包名 | 核心职责 | 真机复现阶段状态 |
|---|---|---|
| `yunyan_bringup` | 键盘遥操作节点 `car_teleop`，发布 `/cmd_vel`；真机建图时用于低速遥控绕场及航点采集 | ✅ 建图/采集航点启用 |
| `yunyan_sim` | Gazebo 仿真世界、差速小车 URDF/xacro 模型、雷达插件与 `sim_bringup.launch.py` | ❌ 仅离线 M1~M3 仿真 |
| `yunyan_nav` | SLAM Toolbox 在线建图参数、Nav2 导航参数、`slam_sim` / `nav2_sim` 启动脚本、APF 避障节点 | ❌ 仅离线 M2/M3 仿真 |
| `yunyan_e2_bridge` | 与 E2 底盘通信的核心：`e2_api_client`、`e2_bridge_node`、`e2_probe`、`e2_waypoint_patrol`、Mock Server、RViz2 监控配置 | ✅ 全程启用核心包 |

### 架构声明

- **工控机环境弃用声明**：原厂选配的 X86 无风扇工控机**默认搭载 Ubuntu 18.04 + ROS 1 Melodic**（手册 4.1.3 节），该操作系统与 ROS 发行版已停止官方维护；**本工程完全不在该老旧环境中运行**。
- **现代跨平台架构**：本工程统一采用 **Ubuntu 22.04 + ROS 2 Humble 上位机**，通过标准网口以 **HTTP RESTful API（`/robot/api/v1.0/...`）**跨平台解耦控制派宝 E2 导航底盘。
- **真机时间戳规范**：真机模式全局一律 **`use_sim_time: False`**。`e2_bringup.launch.py` 中 `robot_state_publisher` 显式声明为 `False`，桥接节点统一使用宿主系统的墙上时钟（Wall Clock），现场严禁启动 Gazebo 或任何 `/use_sim_time` 仿真时钟节点。
- **底盘核心职责划分**：真实的 SLAM 栅格建图、全局重定位、全局路径搜索与底层传感器避障全部由 **E2 底盘内置 Linux 导航模块**独立完成；本工程的上档桥接层负责 ROS 2 标准生态适配（话题/服务映射、TF 广播、RViz2 监控、多点巡航控制以及安全限速与急停保护）。

```text
[Ubuntu 22.04 + ROS 2 Humble 上位机]
  yunyan_bringup  --/cmd_vel-->  yunyan_e2_bridge  --HTTP RESTful (GET/POST/PUT/DELETE)-->  [E2 底盘导航模块]
                                       |
                                       +-- 发布 /e2/odom, /e2/map, /e2/plan, /e2/action_status
                                       +-- 广播 TF: map -> odom -> base_footprint
                                       +-- RViz2 实时可视化 (e2_view.rviz)
  yunyan_sim / yunyan_nav  → 仅离线 M1~M3 仿真验证，真机部署时不启动
```

---

## 2. 派宝 E2 硬件规格与上电安全红线（严格对照手册）

### 2.1 核心硬件参数表

| 参数类别 | 规格项 | 原厂手册标称值 / 规范 |
|---|---|---|
| **机械规格** | 底盘尺寸（长 × 宽） | `500 × 600 mm` |
| **机械规格** | 整机高度 | `1250 mm` |
| **机械规格** | 整机净重 | `32 kg` |
| **机械规格** | 额定载重 / 最大极限载重 | `50 kg` / `60 kg` |
| **通过性能** | 最小通过窄道宽度 | `550 mm`（通道应预留安全裕量） |
| **感知硬件** | 2D 激光雷达最大扫描半径 | `40 m` |
| **感知硬件** | 低矮障碍物识别传感器 | `1` 个，最低可识别障碍高度 `> 3 cm` |
| **感知硬件** | 超声波避障传感器 | `5` 个环形布置，有效识别距离 `45 cm` |
| **建图能力** | 地图栅格分辨率 | `50 mm`（即 `0.05 m/cell`） |
| **建图能力** | 单图最大建图面积 | `2 万平方米` |
| **速度特性** | 原厂出厂默认行走速度 | `0.7 m/s` |
| **速度特性** | 建图模式最大允许行走速度 | `0.6 m/s`（实测推荐更低） |
| **工作环境** | 环境温度 / 防护等级 | `0℃ ~ 40℃` / `IP22`（不防水、防滴水等级） |
| **电气特性** | 动力电池告警临界电压 | 电池电压 `< 22.5 V` 必须立即停机充电 |

> [!WARNING]
> **原厂安全红线声明**：手册明确指出，派宝 E2 机器人底盘**不具备完备的自动防撞、防跌落、生物接近预警等高级安全功能**；任何手动遥控与自主巡航测试均须在视距内、空旷室内平坦场地进行，现场操作者负有最高安全监护职责。

### 2.2 上电前强制安全点检表（Checklist）

现场操作前，请逐条核验并打勾：

- [ ] **电量检查**：开机前确认动力电池电压**不得低于 `22.5 V`**；若低于阈值须先推至充电桩充满后再作业，严禁边充电边建图。
- [ ] **环境要求**：测试区域必须为**室内平坦、干燥且无积水**地面（IP22 防护等级不防水，允许环境温度 `0℃ ~ 40℃`）；清理测试场地上散落的线缆与杂物，全程保持在**操作人员目视范围内**。
- [ ] **机械与操作禁忌**：**严禁在断电状态下强行手推车体**（会破坏电机减速箱齿轮）；手动键盘遥控状态下底盘不具备完整自动防撞与防跌落能力，严禁高速冲撞。
- [ ] **急停与电源确认**：操作前必须**先确认底盘上方机身急停按钮**位置，并顺时针旋转确保其**处于弹出未锁死状态**；打开**底盘底部机械电源开关**，确认车身周围**环形灯带点亮**，表示底盘微系统与雷达完成上电初始化。
- [ ] **软件双重安全限速机制**：
  - `yunyan_e2_bridge` 在 `config/e2_bridge_params.yaml` 中将线速度硬上限严格压制在 **`0.30 m/s`**（出厂标称的 43%）、角速度硬上限压制在 **`0.60 rad/s`**；
  - `car_teleop` 键盘节点每次击键仅下发单步微动（`0.15 m` / `0.25 rad`，最小保护间隔 `0.25 s`）。
- [ ] **保护机制闭环验证**：
  - **下降沿主动刹停**：键盘松手时 `/cmd_vel` 速度归零，桥接节点检测到下降沿后自动调用 `/e2/cancel_navigation`（`DELETE /motion/action`）确保不滑行；
  - **巡航同步撤单**：巡航节点 `e2_waypoint_patrol` 捕获 `Ctrl+C` 信号后，会**同步向底盘发送撤单服务**再行退出。

---

## 3. 上位机环境准备与编译（Ubuntu 22.04 + ROS 2 Humble）

### 3.1 安装系统依赖与 ROS 2 Humble 包

在 Ubuntu 22.04 上位机上打开终端，安装编译与运行所需的完整依赖项：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  python3-requests \
  python3-yaml \
  python3-flake8 \
  python3-pytest
```

### 3.2 编译独立工作空间

本工程 `lab02_sim_slam_nav` 是一个独立的 colcon 工作空间：

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> [!NOTE]
> 在每个新打开的终端中，均需先加载底层环境与本工作空间环境：
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/Auto_Driving/experiments/lab02_sim_slam_nav/install/setup.bash
> ```

---

## 4. 派宝 E2 真机全流程复现指南（严格分步 SOP）

> [!IMPORTANT]
> 执行以下所有命令前，请将 `<E2_IP>` 和 `<Port>` 替换为现场实车的真实局域网 IP 与服务端口。现场不要使用 `127.0.0.1:8080`（除非在离线运行 Mock 仿真服务）。

### 步骤 1：物理接线与网络连通性确认

1. 使用标准千兆网线将上位机网口接入 E2 顶部的 **RJ45 网口母头**；或者将上位机与 E2 连接到同一局域网路由器（E2 支持 Wi-Fi `2.4G/5G` 无密码或 WPA2 认证网络）。
2. **Base URL 空白填补**：原厂手册第 7 页的 `Base URL` 为空白，现场需要通过路由器 DHCP 客户端列表或底盘配置工具核对 E2 实际 IP（常见为 `192.168.1.X` 或 `192.168.11.X`，端口常见为 `8080` 或 `80`）。
3. 验证网络通信连通性：

```bash
# 1. 验证 ICMP 链路低延迟连通
ping -c 4 <E2_IP>

# 2. 验证 HTTP RESTful 服务是否正常响应
curl -i http://<E2_IP>:<Port>/robot/api/v1.0/slam/mode
```

- **验收标准**：`ping` 丢包率为 0% 且 RTT 延迟稳定；`curl` 返回 HTTP 状态码 `200 OK`，且响应 JSON 包含 `{"data": {"mode": ...}}` 字段。若网络未通，请检查网线插头与上位机静态 IP 掩码配置，严禁盲目执行后续步骤。

### 步骤 2：零移动只读接口探针诊断（上车必做）

在**小车静止、急停未触发**的前提下，执行内置的零移动诊断探针：

```bash
ros2 run yunyan_e2_bridge e2_probe --url http://<E2_IP>:<Port>
```

探针会按照只读原则依次向实车发送 6 个核心 GET 请求，评估状态码、往返延迟与字段结构：
1. `GET /robot/api/v1.0/slam/mode` —— 读取当前建图/导航工作模式
2. `GET /robot/api/v1.0/slam/pose` —— 读取机器人当前定位坐标与朝向
3. `GET /robot/api/v1.0/slam/map/info` —— 读取地图尺寸与原点元信息
4. `GET /robot/api/v1.0/slam/map/occupancy-grid` —— 读取实时栅格地图数据
5. `GET /robot/api/v1.0/motion/action` —— 读取导航动作执行器状态
6. `GET /robot/api/v1.0/motion/action/path` —— 读取全局规划路径轨迹

所有原始 JSON 响应将自动转储到 **`/tmp/e2_probe_dump.json`**。

- **验收标准**：终端输出 6 项全部显式标记为 `✔ [OK]`，平均往返延迟在合理范围内（局域网直连通常 `< 50 ms`）。该步骤**完全不调用任何运动指令接口**，确保车体绝对静止安全。

### 步骤 3：启动真机桥接与 RViz2 监控

在新终端中启动真机桥接节点与 RViz2 监控界面：

```bash
ros2 launch yunyan_e2_bridge e2_bringup.launch.py mock:=false base_url:=http://<E2_IP>:<Port>
```

核对项：
- RViz2 自动启动并载入预置视图 `e2_view.rviz`；
- 视口中呈现 1:1 真实尺寸的派宝 E2 3D 模型，模型几何朝向与场地实车一致；
- 查看底盘里程计更新：
  ```bash
  ros2 topic echo /e2/odom --once
  ```
- 查看 TF 坐标变换树（确认 `map -> odom -> base_footprint` 链条正常发布且无仿真时钟警告）：
  ```bash
  ros2 run tf2_ros tf2_echo map base_footprint
  ```
- 查看底盘动作状态话题：
  ```bash
  ros2 topic echo /e2/action_status --once
  ```

- **验收标准**：RViz2 正常显示 E2 模型与 TF 树，`/e2/odom` 位姿数据持续刷新，无任何 `/use_sim_time` 告警。

### 步骤 4：实地建实景图与一键导出

1. **终端 A —— 发起从零清空建图**：

```bash
ros2 service call /e2/start_new_mapping std_srvs/srv/Trigger
```

> **原理解析**：该服务对应底层 `PUT /robot/api/v1.0/slam/mode`，请求载荷为 `{"mode":"0","cleanupMap":true}`，会清空底盘旧图并开启全新建图模式。若需在已有图纸基础上续建，请改用服务 `/e2/switch_to_mapping`（`cleanupMap=false`）。

2. **终端 B —— 遥控小车实景环绕建图**：

在相对空旷、完全在视距内的区域启动键盘控制：

```bash
ros2 run yunyan_bringup car_teleop
```

> **操作规范**：
> - 使用 `W/A/S/D` 进行点动微步移动；
> - 遥控线速度已被桥接层硬锁在 `≤0.30 m/s`（远低于手册最大建图速度 `0.6 m/s`）；
> - 键盘松手后 `/cmd_vel` 下降沿会触发零速度刹停；
> - 遥控小车缓慢绕测绘场地一周，在 RViz2 中实时观测 `/e2/map` 黑色障碍物边界闭合，避免剧烈急转弯产生重影。

3. **终端 C —— 一键导出地图产物**：

建图闭环完成且栅格轮廓稳定后，调用一键导出服务：

```bash
ros2 service call /e2/export_map std_srvs/srv/Trigger
```

- **验收标准**：服务返回 `success=True`，桥接包将在 `experiments/lab02_sim_slam_nav/e2_maps/` 目录下自动生成带时间戳的实地地图三件套：
  - `e2_map_<timestamp>.map`：E2 原厂复合二进制格式地图（可供原厂软件恢复）；
  - `e2_map_<timestamp>.pgm`：标准 ROS 栅格灰度图像；
  - `e2_map_<timestamp>.yaml`：ROS 2 Map Server 格式元数据配置文件（分辨率 `0.05`、原点坐标等）。

> [!NOTE]
> 这些生成的地图文件属于本地导出产物，已被 `.gitignore` 规则屏蔽以防大图污染代码库。若仅需底盘内部固化而不下传上位机，可调用 `ros2 service call /e2/save_map std_srvs/srv/Trigger`。

### 步骤 5：切换导航定位模式与安全采集真实航点

1. **切换到底盘导航模式**：

```bash
ros2 service call /e2/switch_to_navigation std_srvs/srv/Trigger
```

> 对应底层 `PUT /robot/api/v1.0/slam/mode`（`mode="1"`）。
> 若小车初始位姿与地图存在偏差，可在 RViz2 顶部工具栏点击 **2D Pose Estimate**，在小车当前实际所处位置拉出朝向箭头，桥接节点会转换为 `PUT /robot/api/v1.0/slam/pose`（携带 `mode: "assist"`）通知底盘激光辅助完成重定位。

2. **近距离单点试跑**：
在 RViz2 中使用 **2D Goal Pose** 工具在小车前方 **1.5 m 以内**的绝对空旷处设定一个导航目标，观察小车平稳启动、减速停正，确认 `/e2/action_status` 从 `EXECUTING` 转为 `SUCCEEDED`。

> [!CAUTION]
> **强制安全警告：严禁直接使用默认 Mock 航点文件 `e2_waypoints.yaml` 驱动 32 kg 实车！**
> 该示例文件中的坐标（如 `(1.5, 0.0)`, `(2.0, 1.5)`, `(0.0, 1.5)`）仅为离线模拟环境虚拟数值，未对照实车测试场地的真实障碍物！

3. **采集真实安全航点规范**：
- 使用键盘遥操作 `car_teleop` 将小车小心遥控至现场确认平坦、安全的第一个目标停驻点；
- 执行命令读取当前高精度位姿：
  ```bash
  ros2 topic echo /e2/odom --once
  ```
- 记录输出中的 `pose.pose.position.x` 和 `pose.pose.position.y`（以及 `orientation` 对应的 `yaw` 角）；
- 重复上述步骤采集 2~3 个确认安全的巡航点，新建并保存为自定义航点文件（例如 `real_waypoints.yaml`）：

```yaml
# 派宝 E2 真机实测安全航点集（坐标必须源自现场实地采集）
loops: 1
goal_timeout_sec: 35.0
dwell_sec: 2.5
retry_on_fail: 0

waypoints:
  - name: "safe_checkpoint_1"
    x: 0.85          # 替换为现场实测 x 坐标 (m)
    y: 0.00          # 替换为现场实测 y 坐标 (m)
    yaw: 0.0         # 期望朝向角度 (rad)
  - name: "safe_checkpoint_2"
    x: 1.30          # 建议初次测试距离控制在 1.5m 范围内
    y: 0.20
    yaw: 0.52
```

- **验收标准**：航点文件内所有坐标均来自实车回传数据，各航点间视线通畅且无盲区障碍物。

### 步骤 6：多点自主巡航与动态避障验证

1. **执行多点巡航任务**：

```bash
ros2 run yunyan_e2_bridge e2_waypoint_patrol \
  --ros-args -p waypoints_file:=/home/lifeline/Auto_Driving/experiments/lab02_sim_slam_nav/real_waypoints.yaml
```

巡航节点将依次下发目标，监听 `/e2/action_status` 判断到位、到达后静止驻留 `dwell_sec`，然后推进下一航点。

2. **多传感器动态避障验收方法**：
- 在巡航预定直线轨迹前方约 **1.5 m** 处，人工放置一个高度 **`> 3 cm`** 的常规硬纸箱（模拟地面低矮凸起障碍）；
- 观察派宝 E2 底盘内置感知链路（40m 激光雷达 + 1 路低矮传感器 + 5 路超声波）融合触发局部避障规划；
- 在 RViz2 中观察 `/e2/plan` 路径平滑重新规划并绕开纸箱，小车在保持安全裕量的前提下绕障并最终到达预定航点；
- 话题 `/e2/action_status` 成功转为 `SUCCEEDED`。

3. **应急刹停与收尾关机**：
- 巡航过程中任意时刻按下 **`Ctrl+C`**，巡航节点会拦截终止信号并**立即向底盘调用 `/e2/cancel_navigation`** 撤单刹停；
- 若出现紧急状况，可直接在上位机终端调用急停服务：
  ```bash
  ros2 service call /e2/cancel_navigation std_srvs/srv/Trigger
  ```
  或者**直接拍下机身物理急停按钮**。
- 测试完成后，弯腰关闭**底盘底部电源开关**，确认环形灯带熄灭，完成全部现场实验。

---

## 5. ROS 2 接口与 E2 RESTful API 映射速查 + 离线仿真/Mock 命令

### 5.1 通信接口映射速查表

| ROS 2 话题 / 服务 | 数据流向 | 对应 E2 RESTful API 路由（基准前缀 `/robot/api/v1.0`） | 语义与负载说明 |
|---|---|---|---|
| `/goal_pose` | 上位机 → 底盘 | `POST /motion/action` | 下发自主导航终点坐标 `{x, y, yaw}` |
| `/initialpose` | 上位机 → 底盘 | `PUT /slam/pose` | 辅助重定位 `{mode: "assist", x, y, yaw}` |
| `/cmd_vel` | 上位机 → 底盘 | `POST /motion/rectilinear` 或 `POST /motion/rotation` | 遥控线速/角速微步，受 `0.30 m/s` / `0.60 rad/s` 约束 |
| `/e2/odom` | 底盘 → 上位机 | `GET /slam/pose` | 轮询底盘位姿并发布 `nav_msgs/Odometry` 及 TF |
| `/e2/map` | 底盘 → 上位机 | `GET /slam/map/occupancy-grid` | 轮询栅格地图并发布 `nav_msgs/OccupancyGrid` |
| `/e2/plan` | 底盘 → 上位机 | `GET /motion/action/path` | 轮询全局规划轨迹并发布 `nav_msgs/Path` |
| `/e2/action_status` | 底盘 → 上位机 | `GET /motion/action` | 轮询导航状态并发布为标准字符串状态 |
| `/e2/start_new_mapping` | 服务接口 | `PUT /slam/mode` | 清空旧图开启新图 `{mode: "0", cleanupMap: true}` |
| `/e2/switch_to_mapping` | 服务接口 | `PUT /slam/mode` | 保持现有图层继续建图 `{mode: "0", cleanupMap: false}` |
| `/e2/switch_to_navigation` | 服务接口 | `PUT /slam/mode` | 切换到导航定位模式 `{mode: "1", cleanupMap: false}` |
| `/e2/cancel_navigation` | 服务接口 | `DELETE /motion/action` | 撤销当前导航任务，底盘紧急制动 |
| `/e2/save_map` | 服务接口 | `POST /slam/map` | 请求底盘固化保存当前地图 `{mapName}` |
| `/e2/export_map` | 服务接口 | `POST /slam/map` + `GET /slam/map` + `GET /slam/map/occupancy-grid` | 触发底盘保存并一键下载二进制 `.map` 及 ROS `.pgm`/`.yaml` |
| `e2_probe` (CLI) | 诊断工具 | 6 个只读 GET 接口批量测试 | 零移动连通性与数据字段探针，结果落盘 `/tmp/` |

### 5.2 离线无真机快速复现指南

无 E2 实车硬件时，可利用工程自带的全功能 Mock 服务或 Gazebo 仿真环境完成技术栈复现验证：

#### 方案一：E2 HTTP 桥接离线 Mock 联调（包含内置 HTTP Server + 10 Hz 动力学插值 + RViz2）

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
source /opt/ros/humble/setup.bash && source install/setup.bash

# 一键启动 Mock 模式与 RViz2 监控（无需真机硬件）
ros2 launch yunyan_e2_bridge e2_bringup.launch.py mock:=true
```

另开终端测试航点巡航或键盘微动：
```bash
ros2 run yunyan_e2_bridge e2_waypoint_patrol
```

#### 方案二：阶段 M1 ~ M3 Gazebo 动力学仿真与 Nav2 算法栈

```bash
# M1：Gazebo 差速底盘与 2D 雷达仿真世界 + 键盘遥操作
ros2 launch yunyan_sim sim_bringup.launch.py
ros2 run yunyan_bringup car_teleop

# M2：SLAM Toolbox 在线异步图优化建图与存图
ros2 launch yunyan_nav slam_sim.launch.py
ros2 run nav2_map_server map_saver_cli -f src/yunyan_nav/maps/slam_room_map

# M3-方案 B：APF 人工势场轻量级反应式避障节点
ros2 run yunyan_nav apf_avoidance

# M3-方案 A：Nav2 工业级已知地图自主导航
ros2 launch yunyan_nav nav2_sim.launch.py

# M3-方案 A：在线实时 SLAM + Nav2 动态建图导航联调
ros2 launch yunyan_nav nav2_sim.launch.py slam:=True
```

---

## 6. 真机常见故障排查 Q&A（融合手册第 6 章与桥接诊断）

### Q1：发送导航目标或遥控指令后，机器人完全不动？
- **排查步骤 1（急停按钮）**：首先检查机器人顶部机身的**红色物理急停按钮**是否处于按下状态。若被按下，顺时针旋转使其弹起复位；
- **排查步骤 2（状态监控）**：在终端查看 `ros2 topic echo /e2/action_status --once`，核对状态是否为 `FAILED` 或 `CANCELED`；
- **排查步骤 3（模式检查）**：确认底盘是否已成功切入导航模式（`mode=1`），若仍处于空闲或错误模式，重新调用 `/e2/switch_to_navigation`；
- **排查步骤 4（网络排查）**：执行 `e2_probe`，检查是否因为网线松动导致 RESTful 请求超时。

### Q2：运行中小车持续发出连续的“滴-滴-滴...”蜂鸣报警声？
- **诊断**：手册第 6 章明确指出，该报警音表明动力电池已触及低电量警戒线（低于 `22.5 V`）。
- **处理规范**：严禁强制继续作业，必须**立即停机并将小车推至原厂充电桩进行补充充电**；若电池电压高于 `24 V` 仍然持续报警，说明底盘内置控制器检测到电机过载或传感器通信异常硬件错误码，应读取底盘错误码并联系映博智能售后技术支持，严禁私自拆卸底盘外壳。

### Q3：小车导航目的地存在偏移，或在地图中发生定位发散？
- **诊断**：可能由于场地光照剧烈变化、玻璃幕墙反射或开机初始位姿与原图差异过大导致激光匹配失败。
- **处理规范**：将小车安全推靠/移回充电桩（充电桩为全局最高置信度几何锚点）或已知显著特征点；在 RViz2 顶部点击 **2D Pose Estimate** 重新拉出初始估计朝向，触发底盘的 `mode: "assist"` 激光辅助重定位，直到雷达点云与栅格边界完全吻合后再下发导航点。

### Q4：RViz2 中 `/e2/map` 地图未显示或 `/e2/plan` 路径无更新？
- **诊断**：可能是实车固件 API 返回的 JSON 字段格式与通用协议存在差异（例如嵌套层级不同或栅格编码方式差异）。
- **处理规范**：查看 `/tmp/e2_probe_dump.json`，核验实车返回的原始 JSON 数据；当前 `e2_bridge_node` 已全面兼容 Base64 栅格编码、二维数值矩阵以及嵌套的 `map/points` 路径格式；若发现实车返回了新字段，请将 dump 文件提取后补充适配解析逻辑。

### Q5：充电无法正常进行或充电器指示灯状态异常？
- **诊断与处理**：
  - 正常状态下充电器接入电源且为底盘充电时，指示灯应常亮**红色**；
  - 若充电器连接至充电桩底座后出现**红绿交替闪烁**异常，必须先切断充电器市电插头并关闭机器人电源开关；
  - 使用干净干燥的无尘布仔细清理机器人底部的两枚金属充电触片以及充电桩上的金属弹性触针，去除氧化层或粉尘后再重新尝试；
  - 确认作业环境温度高于 `0℃`，严禁在冰冻潮湿环境下违规充电。

---

## 附：工程提交与代码规范

- **提交信息规范**：遵循 Conventional Commits 格式，本阶段标准提交信息为：
  ```text
  docs(lab02): add PadBot E2 real-robot deployment README and clean mock artifacts
  ```
- **工作分支**：`feature/lab02_e2_bridge`
- **代码规范检查**：
  ```bash
  cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
  python3 -m flake8 src/
  ```
  保证源码风格保持 0 错误（PEP 8 / ROS 2 Python 标准规范）。

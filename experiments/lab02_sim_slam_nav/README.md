# 派宝 E2 真机复现与建图导航实操手册

> **核心定位**：本手册面向 **广州映博智能科技有限公司 PadBot E2（派宝 E2）模块化开发平台**。针对现场拿到实车后的实操人员，严格对照原厂《E2 用户手册》（文本规范见仓库根目录 [`e2_ver_1_0_1.md`](../../e2_ver_1_0_1.md)）与 ROS 2 Humble `yunyan_e2_bridge` 桥接工程，提供从硬件上电点检、零移动接口探针诊断、实地 SLAM 建图与出图，到单点/多点自主导航及动态避障验证的完整标准作业程序（SOP）。

---

## 1. 项目简介与技术路线

本工程旨在实现 Ubuntu 22.04 上位机对派宝 E2 实体机器人底盘的**物理状态监控、实地建图与出图、单点/多点自主巡航导航以及动态融合避障**。

### 现代跨平台技术路线说明

- **工控机老旧环境彻底弃用**：原厂选配的无风扇工控机出厂预装 Ubuntu 18.04 + ROS 1 Melodic（手册 4.1.3 节），该软件栈已停止维护。**本工程完全不采用该环境**。
- **解耦的 HTTP RESTful 架构**：派宝 E2 底盘内置 Linux 导航模块（提供 `/robot/api/v1.0/...` RESTful API）。外部上位机（支持任意预装 **Ubuntu 22.04 + ROS 2 Humble** 的笔记本电脑或工控设备）通过标准以太网网线或局域网 Wi-Fi，直接与 E2 底盘高速交互。
- **真机时间戳规范**：真机复现模式下全局一律声明 **`use_sim_time: False`**。`e2_bringup.launch.py` 中的 `robot_state_publisher` 与桥接节点统一使用系统墙上时钟（Wall Clock），现场严禁启动任何仿真时钟。
- **底盘核心职责划分**：真实的 SLAM 栅格建图、全局重定位、全局路径搜索与底层多传感器融合避障由 **E2 底盘内置导航系统**闭环完成；上位机 `yunyan_e2_bridge` 负责 ROS 2 标准生态桥接（发布 `/e2/odom`、`/e2/map`、`/e2/plan`、`/e2/action_status`，广播 `map -> base_footprint` TF 树，集成 RViz2 可视化监控与工程级软限速防撞保护）。

```text
[Ubuntu 22.04 + ROS 2 Humble 上位机]
  teleop_twist_keyboard  --/cmd_vel-->  yunyan_e2_bridge  --HTTP RESTful (GET/POST/PUT/DELETE)-->  [E2 底盘导航模块]
                                             |
                                             +-- 发布 /e2/odom, /e2/map, /e2/plan, /e2/action_status
                                             +-- 广播 TF: map -> base_footprint
                                             +-- RViz2 1:1 三维监控 (e2_view.rviz)
```

---

## 2. 派宝 E2 硬件规格与安全操作红线（严格对照手册）

### 2.1 核心硬件参数表（对照《E2用户手册》1.1 ~ 1.3 节）

| 参数类别 | 规格参数项 | 原厂标称参数 / 规范要求 |
|---|---|---|
| **机械规格** | 底盘轮廓尺寸（长 × 宽） | `500 × 600 mm` |
| **机械规格** | 整机高度 | `1250 mm` |
| **机械规格** | 整机净重 | `32 kg` |
| **机械规格** | 额定负重 / 最大极限负重 | `50 kg` / `60 kg` |
| **驱动轮系** | 驱动电机 / 从动轮规格 | 双 6 寸轮毂电机 + 双 3 寸万向从动轮 |
| **通过性能** | 最小通过窄道宽度 | `550 mm`（现场通道建议预留裕量） |
| **通过性能** | 垂直过坎 / 水平过沟壑高度 | `30 mm` / `30 mm` |
| **通过性能** | 最大爬坡角度 | `5° ~ 10°` |
| **感知硬件** | 2D 激光雷达扫描半径 | `40 m` |
| **感知硬件** | 低矮障碍物识别传感器 | `1` 个，最低有效识别高度 `> 3 cm` |
| **感知硬件** | 超声波避障传感器 | `5` 个环形布置，有效识别距离 `45 cm` |
| **建图指标** | 地图栅格分辨率 / 最大建图面积 | `50 mm`（`0.05 m/cell`） / `20,000 ㎡`（2 万平方米） |
| **运动速度** | 原厂出厂默认行走速度 | `0.7 m/s` |
| **运动速度** | 建图模式最大允许速度 | `0.6 m/s`（实测建议 `≤ 0.3 m/s`） |
| **电池动力** | 动力电池规格与续航 | `12800 mAh` 磷酸铁锂电池，续航 `> 10 H` |
| **电气特性** | 充电时长 / 告警临界电压 | 标准桩充电 `9 h` / 电池电压 `< 22.5 V` 必须立即停机充电 |
| **通信接口** | 上部拓展物理接口 | **RJ45 网口母头**、Wi-Fi（2.4GHz / 5GHz）、BLE 5 |
| **工作环境** | 环境温度 / 防护等级 | `0℃ ~ 40℃` / `IP22`（不防水、防垂直落滴） |

> [!WARNING]
> **原厂安全红线声明**：手册第 1.0 节明确指出，派宝 E2 底盘**不具备完整的自动防撞、防跌落、生物接近预警等安全功能**；任何遥控与自主巡航测试均须在视距内、空旷平坦场地进行，现场操作人员负有不可推卸的监护安全责任。

### 2.2 上电前强制安全清单（Checklist）

每次真机开机前，操作者须逐项严格确认：

- [ ] **电量与充电安全**：确认动力电池电压**不得低于 `22.5 V`**；若低于阈值须立即使用原厂充电器充电；充电环境温度必须 `> 0℃`，严禁使用非原厂充电器，严禁边充电边建图。
- [ ] **场地环境**：仅限在**室内平坦、干燥且无积水**地面运行（IP22 防护等级不防水，环境温度 `0℃ ~ 40℃`）；清理测试场地上散落的线缆与杂物，全程保持在**操作人员目视范围内**。
- [ ] **机械操作禁忌**：**严禁在断电状态下强行手推车体**（会损坏电机内部减速箱齿轮）；纯键盘遥控模式下底盘不具备完整自动防撞与防跌落功能，必须慢速轻点。
- [ ] **开机顺序与急停确认**：
  1. 环视检查车体外观无机械变形与线缆脱落；
  2. 确认**机器人机身急停按钮**位置，并顺时针旋转确保其处于**弹出未锁死状态**；
  3. 打开**底盘底部机械电源开关**；
  4. 确认车身周围**环形灯带点亮**，表示底盘微系统与激光雷达完成上电初始化。
- [ ] **工程级软件双重安全保护**：
  - `yunyan_e2_bridge` 在 `config/e2_bridge_params.yaml` 中将线速度硬上限压制在 **`0.30 m/s`**、角速度硬上限压制在 **`0.60 rad/s`**（远低于原厂的 `0.7 m/s`）；
  - **下降沿主动刹停**：键盘松手 `/cmd_vel` 速度归零时，桥接节点检测到下降沿后自动调用 `/e2/cancel_navigation`（`DELETE /motion/action`）确保不滑行；
  - **巡航同步撤单**：巡航节点 `e2_waypoint_patrol` 捕获 `Ctrl+C` 信号后，会**同步向底盘发送撤单制动服务**再行退出。

---

## 3. 上位机环境准备与单包编译（Ubuntu 22.04 + ROS 2 Humble）

本工程完全剥离了 Gazebo 仿真世界、云衍小车旧固件以及外部 Nav2 / SLAM Toolbox 等冗余包，仅需安装 E2 真机必需依赖。

### 3.1 安装系统与 ROS 2 必需依赖项

在 Ubuntu 22.04 上位机打开终端，执行以下命令安装核心依赖：

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

### 3.2 独立编译 `yunyan_e2_bridge` 单包

进入工作空间，直接对 `yunyan_e2_bridge` 执行单包构建：

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
source /opt/ros/humble/setup.bash
colcon build --packages-select yunyan_e2_bridge --symlink-install
source install/setup.bash
```

> [!NOTE]
> 在后续每次新打开的终端中，均需执行环境初始化：
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/Auto_Driving/experiments/lab02_sim_slam_nav/install/setup.bash
> ```

---

## 4. 派宝 E2 真机复现全流程 SOP（核心章节）

> [!IMPORTANT]
> 执行以下所有命令前，请将 `<E2_IP>` 和 `<Port>` 替换为现场实车的真实局域网 IP 与服务端口（如 `192.168.1.100:8080`）。现场切勿盲目使用 `127.0.0.1:8080`（除非在离线运行 Mock 模拟服务）。

### 步骤 1：物理接线与网络验通

1. 按照原厂手册 3.3.1 节，使用标准千兆网线将上位机网口接入 E2 顶部引出的 **RJ45 网口母头**；或者将上位机与 E2 连接到同一局域网路由器（E2 支持 2.4GHz / 5GHz Wi-Fi）。
2. **Base URL 空白填补**：手册 3.3.2 节的 Base URL 原文留白，现场必须先向网络管理员或通过路由器 DHCP 客户端列表核对 E2 实际分配的 `<E2_IP>:<Port>`。
3. 执行网络双向验通：

```bash
# 1. 验证 ICMP 链路低延迟连通
ping -c 4 <E2_IP>

# 2. 验证 HTTP RESTful 服务是否正常响应
curl -i http://<E2_IP>:<Port>/robot/api/v1.0/slam/mode
```

- **预期现象**：`ping` 丢包率为 0% 且延迟稳定；`curl` 返回 HTTP 状态码 `200 OK`，且响应 JSON 包含 `{"data": {"mode": ...}}`。若网络不通，请检查物理网线与上位机 IP 掩码配置，严禁盲目进入下一步。

### 步骤 2：只读接口探针诊断（零移动，上车必跑）

在**小车静止、急停未触发**的前提下，执行内置的只读诊断探针：

```bash
ros2 run yunyan_e2_bridge e2_probe --url http://<E2_IP>:<Port>
```

探针会依次发送 6 个核心 GET 请求，核查接口状态、响应延迟与字段结构：
1. `GET /robot/api/v1.0/slam/mode` —— 读取当前建图/导航工作模式
2. `GET /robot/api/v1.0/slam/pose` —— 读取机器人当前定位坐标与朝向
3. `GET /robot/api/v1.0/slam/map/info` —— 读取地图尺寸与原点元信息
4. `GET /robot/api/v1.0/slam/map/occupancy-grid` —— 读取实时栅格地图数据
5. `GET /robot/api/v1.0/motion/action` —— 读取导航动作执行器状态
6. `GET /robot/api/v1.0/motion/action/path` —— 读取全局规划路径轨迹

所有接口返回的原始 JSON 响应将自动转储到 **`/tmp/e2_probe_dump.json`**。

- **预期现象**：终端输出 6 个接口全部显式标为 `✔ [OK]`，平均往返延迟通常 `< 50 ms`。该步骤**完全不调用任何运动指令接口**，确保车体绝对静止安全。

### 步骤 3：启动真机桥接与 RViz2 监控

在新终端中启动真机桥接节点与 RViz2 可视化监控：

```bash
ros2 launch yunyan_e2_bridge e2_bringup.launch.py mock:=false base_url:=http://<E2_IP>:<Port>
```

打开另一终端，核验核心话题与 TF 坐标树：
```bash
# 检查底盘里程计更新
ros2 topic echo /e2/odom --once

# 检查 TF 坐标变换树（map -> base_footprint）
ros2 run tf2_ros tf2_echo map base_footprint
```

- **预期现象**：
  - RViz2 自动弹出并载入预置视图 `e2_view.rviz`；
  - 视口呈现 1:1 比例的 E2 三维模型（`500 × 600 × 1250 mm`），车头朝向与实车完全一致；
  - `/e2/odom` 位姿数据持续刷新，TF 树链条完整发布，终端无任何 `/use_sim_time` 告警。

### 步骤 4：实地建实景图与一键导出

1. **终端 A —— 发起从零清空建图**：

```bash
ros2 service call /e2/start_new_mapping std_srvs/srv/Trigger
```

> **原理解析**：对应底层 `PUT /robot/api/v1.0/slam/mode`（请求体 `{"mode":"0","cleanupMap":true}`），清空底盘旧图并开启全新建图模式。若需在已有图纸基础上续建，请调用 `/e2/switch_to_mapping`（`cleanupMap=false`）。

2. **终端 B —— 启动 ROS 2 标准键盘遥控慢速绕场**：

在相对空旷、完全在视距内的区域启动标准键盘遥操作：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
```

> **操作规范与安全警示**：
> - **键盘制动**：按 **`k` 键立即急停**，切勿仅靠松开方向键期待瞬停（桥接层已内置 `1.0s` 看门狗兜底刹停）；
> - **按键方向**：仅使用 `i/,/j/l` 进行纯直行/纯旋转点动微步移动，**请勿使用 `u/o/m/.` 斜向组合键**（E2 底盘将直行与旋转拆分为独立接口，若输入斜向键将优先执行主导轴并记录提示）；
> - **限速生效**：`-p speed:=0.2 -p turn:=0.4` 依赖 ROS 2 Humble 较新版本 `teleop_twist_keyboard`（>=2.4.1），若旧版本参数未生效可在启动后在键盘界面按 **`z` 键降速**（桥接层内部亦有 `0.30 m/s` 硬限幅兜底）；
> - **重要安全警示**：`max_linear_vel: 0.30` 仅限制 `/cmd_vel` 键盘遥控指令；后续通过 `/goal_pose` 或 `e2_waypoint_patrol` 下发的自主导航任务由 E2 底盘按出厂默认速度（手册标称最高 `0.7 m/s`）全速执行，**实车测试时务必保持场地空旷并预先安排人员手扶物理急停按钮**；
> - 遥控小车慢速环绕测绘场地一周，在 RViz2 中实时观测 `/e2/map` 黑色障碍物轮廓闭合，避免高速急转弯产生重影。

3. **终端 C —— 一键保存并出图**：

建图闭环完成且栅格轮廓稳定后，调用一键导出服务：

```bash
ros2 service call /e2/export_map std_srvs/srv/Trigger
```

- **预期现象**：服务返回 `success=True`，桥接包会在 `experiments/lab02_sim_slam_nav/e2_maps/` 目录下自动生成实地地图三件套：
  - `e2_real_scene_<timestamp>.map`：E2 原厂复合二进制地图文件；
  - `e2_real_scene_<timestamp>.pgm`：标准 ROS 栅格灰度图像（可直接图片浏览器预览）；
  - `e2_real_scene_<timestamp>.yaml`：ROS 2 Map Server 格式元数据文件（分辨率 `0.05`、原点坐标等）。

> [!NOTE]
> 导出的地图文件已被 `.gitignore` 规则屏蔽以防大图污染仓库。若仅需底盘内部保存而不下载到上位机，可调用 `ros2 service call /e2/save_map std_srvs/srv/Trigger`。

### 步骤 5：切换导航定位模式、辅助重定位与单点自主导航

1. **切换到底盘导航模式**：

```bash
ros2 service call /e2/switch_to_navigation std_srvs/srv/Trigger
```

> 对应底层 `PUT /robot/api/v1.0/slam/mode`（`mode="1"`）。

2. **辅助重定位校准**：
若小车当前物理位置与 RViz2 显示位姿存在偏差，在 RViz2 顶部工具栏点击 **2D Pose Estimate**，在地图中小车实际所处位置沿其实际朝向拉出绿色箭头。桥接节点将触发底层 `PUT /robot/api/v1.0/slam/pose`（携带 `mode: "assist"`），底盘将利用激光雷达点云与已知栅格做关联匹配完成辅助重定位。

3. **近距离单点自主导航测试**：
在 RViz2 工具栏点击 **2D Goal Pose**，在小车前方 **1.5 m 以内**的绝对空旷安全处点击设定导航终点。
- **预期现象**：
  - `/e2/plan` 路径绿色折线实时规划生成；
  - 小车平稳启动、行进、减速并精准停正；
  - 执行状态话题 `/e2/action_status` 从 `EXECUTING` 顺利转变为 `SUCCEEDED`。

### 步骤 6：采集真实航点、多点自主巡航与动态避障验证

> [!CAUTION]
> **强制安全警告：严禁直接使用默认 Mock 航点文件 `config/e2_waypoints.yaml` 驱动 32 kg 实车！**
> 该文件中的坐标（如 `(1.5, 0.0)`, `(2.0, 1.5)`, `(0.0, 1.5)`）仅为离线调试虚拟数值，未对照实车场地的真实物理障碍物！

#### 1. 实车采集真实安全航点规范
- 使用键盘遥控将小车开到现场确认平坦、安全的第一个目标停驻点并停稳；
- 在上位机终端读取当前实测坐标：
  ```bash
  ros2 topic echo /e2/odom --once
  ```
- 记录输出中的 `pose.pose.position.x` 和 `pose.pose.position.y`（以及期望朝向角 `yaw`）；
- 重复上述步骤采集 2~3 个确认安全的巡航点，保存为自定义文件（如 `real_waypoints.yaml`）：

```yaml
# 派宝 E2 真机现场采集安全航点文件
loops: 1
goal_timeout_sec: 35.0
dwell_sec: 2.5
retry_on_fail: 0

waypoints:
  - name: "checkpoint_1"
    x: 0.85          # 替换为实地采集 x 坐标 (m)
    y: 0.00          # 替换为实地采集 y 坐标 (m)
    yaw: 0.0         # 期望朝向角度 (rad)
  - name: "checkpoint_2"
    x: 1.30          # 建议初次测试距离控制在 1.5m 范围内
    y: 0.20
    yaw: 0.52
```

#### 2. 启动多点自主巡航任务

确保配置文件保存在 `experiments/lab02_sim_slam_nav/real_waypoints.yaml`，并在该目录下运行：

```bash
ros2 run yunyan_e2_bridge e2_waypoint_patrol \
  --ros-args -p waypoints_file:=$(pwd)/real_waypoints.yaml
```

#### 3. 多传感器动态避障验收
- 在巡航预定直线轨迹前方约 **1.5 m** 处，人工放置一个高度 **`> 3 cm`** 的常规硬纸箱（模拟地面低矮凸起障碍物）；
- **预期现象**：
  - 派宝 E2 底盘内置感知系统（40m 激光雷达 + 1 路低矮传感器 + 5 路超声波）融合识别到障碍；
  - RViz2 中的 `/e2/plan` 路径折线动态绕开纸箱；
  - 小车平滑绕过纸箱并继续前行抵达目的地，`/e2/action_status` 转为 `SUCCEEDED`。

#### 4. 紧急制动刹停与收尾关机
- 巡航过程中任意时刻在终端按下 **`Ctrl+C`**，巡航节点会拦截退出信号并**同步调用 `/e2/cancel_navigation`** 实施主动撤单刹停；
- 若现场发生突发状况，亦可直接在终端调用急停服务：
  ```bash
  ros2 service call /e2/cancel_navigation std_srvs/srv/Trigger
  ```
  或者**直接拍下小车机身红色物理急停按钮**。
- 实验完毕确认小车停稳后，弯腰关闭**底盘底部机械电源开关**，确认环形灯带熄灭。

---

## 5. ROS 2 与 E2 RESTful API 映射速查表 + 无真机离线自检

### 5.1 通信接口映射速查表（对照手册 3.3.2 节）

| ROS 2 话题 / 服务 | 数据流向 | 对应 E2 RESTful API（基准前缀 `/robot/api/v1.0`） | 语义与负载说明 |
|---|---|---|---|
| `/goal_pose` | 上位机 → 底盘 | `POST /motion/action` | 下发自主导航终点坐标 `{x, y, yaw}` |
| `/initialpose` | 上位机 → 底盘 | `PUT /slam/pose` | 辅助重定位 `{mode: "assist", x, y, yaw}` |
| `/cmd_vel` | 上位机 → 底盘 | `POST /motion/rectilinear` 或 `POST /motion/rotation` | 遥控线速/角速微步，受 `0.30 m/s` / `0.60 rad/s` 限幅 |
| `/e2/odom` | 底盘 → 上位机 | `GET /slam/pose` | 轮询底盘位姿并发布 `nav_msgs/Odometry` 及 TF |
| `/e2/map` | 底盘 → 上位机 | `GET /slam/map/occupancy-grid` | 轮询栅格地图并发布 `nav_msgs/OccupancyGrid` |
| `/e2/plan` | 底盘 → 上位机 | `GET /motion/action/path` | 轮询全局规划轨迹并发布 `nav_msgs/Path` |
| `/e2/action_status` | 底盘 → 上位机 | `GET /motion/action` | 轮询导航状态并发布为标准状态字符串 |
| `/e2/start_new_mapping` | 服务接口 | `PUT /slam/mode` | 清空旧图开启新图 `{mode: "0", cleanupMap: true}` |
| `/e2/switch_to_mapping` | 服务接口 | `PUT /slam/mode` | 保持现有图层继续建图 `{mode: "0", cleanupMap: false}` |
| `/e2/switch_to_navigation` | 服务接口 | `PUT /slam/mode` | 切换到导航定位模式 `{mode: "1", cleanupMap: false}` |
| `/e2/cancel_navigation` | 服务接口 | `DELETE /motion/action` | 撤销当前导航任务，底盘紧急制动 |
| `/e2/save_map` | 服务接口 | `POST /slam/map` | 请求底盘固化保存当前地图 `{mapName}` |
| `/e2/export_map` | 服务接口 | `POST /slam/map` + `GET /slam/map` + `GET /slam/map/occupancy-grid` | 触发底盘保存并一键下载二进制 `.map` 及 ROS `.pgm`/`.yaml` |
| `e2_probe` (CLI) | 诊断工具 | 6 个只读 GET 接口批量测试 | 零移动连通性与数据字段探针，结果转储 `/tmp/` |

### 5.2 无真机离线 Mock 冒烟自检命令

在无 E2 实体小车的离线环境下，可利用内置的轻量 Mock 服务（集成 HTTP Server + 10 Hz 动力学插值模拟 + RViz2 监控）完成技术栈自检：

```bash
cd ~/Auto_Driving/experiments/lab02_sim_slam_nav
source /opt/ros/humble/setup.bash && source install/setup.bash

# 启动离线 Mock 模式与 RViz2 监控
ros2 launch yunyan_e2_bridge e2_bringup.launch.py mock:=true
```

另开终端测试航点巡航或键盘微动：
```bash
ros2 run yunyan_e2_bridge e2_waypoint_patrol
```

---

## 6. 真机常见问题与故障排查 Q&A（严格对照手册第 6 章）

### Q1：发送导航目标或遥控指令后，机器人完全不移动？
- **排查步骤 1（急停按钮）**：首先检查机身顶部的**红色物理急停按钮**是否被意外按下。若被按下，顺时针旋转使其弹起复位；
- **排查步骤 2（底盘电源与灯带）**：确认底盘底部机械开关已开启，且车身周围**环形灯带常亮**；
- **排查步骤 3（状态监控）**：在终端查看 `ros2 topic echo /e2/action_status --once`，核对状态是否为 `FAILED` 或 `CANCELED`；
- **排查步骤 4（网络连通性）**：执行 `ping <E2_IP>` 与 `e2_probe`，检查是否因为网线松动导致 HTTP 请求超时。

### Q2：运行中小车持续发出连续的“滴-滴-滴...”提示音？
- **原厂诊断**：手册第 6 章明确指出，该连续提示音表示动力电池已进入低电量警报状态（电压低于 `22.5 V`）。
- **处理规范**：严禁强制继续作业，**立即停机并将小车移回原厂充电桩补充充电**（应优先通过键盘低速遥控移回；**严禁在断电状态下强行硬推车体**以免损坏电机内部减速箱齿轮，确需人工移动须确认电机处于释放状态并轻缓推行）；若电池电压充足仍报警，说明底盘内置控制器检测到底层硬件或通信错误码，应读取底盘错误码并联系映博智能售后技术支持，严禁私自拆卸底盘。

### Q3：小车导航目的地存在偏移，或在地图中发生定位发散？
- **原厂诊断**：可能由于测试场地环境光照剧烈变化、玻璃幕墙漫反射或开机初始位姿与地图偏差过大导致激光匹配失败。
- **处理规范**：将小车安全移回充电桩（优先低速遥控，严禁断电强推；充电桩为全局最高置信度几何锚点）或已知显著特征点；在 RViz2 顶部点击 **2D Pose Estimate** 重新标定初始朝向，触发底盘的 `mode: "assist"` 激光辅助重定位，直到雷达点云与栅格边界完全重合后再下发导航点。

### Q4：无法自动充电或充电指示灯状态异常？
- **原厂诊断与处理**：
  - 正常状态下充电器接入电源且为底盘充电时，指示灯应常亮**红色**；
  - 若充电器连接至充电桩底座后出现**红绿交替闪烁**异常，必须先切断充电器市电插头并关闭机器人电源开关；
  - 使用干净干燥的无尘布仔细清理机器人底部的两枚金属充电触片以及充电桩上的金属弹性触针，去除表面氧化层或灰尘后再重新尝试；
  - 确认作业环境温度高于 `0℃`，严禁在冰冻潮湿环境下违规充电。

### Q5：RViz2 中 `/e2/map` 或 `/e2/plan` 未渲染？
- **诊断**：可能是现场实车固件 API 返回的 JSON 字段格式与通用协议存在差异（例如嵌套层级不同或栅格编码方式差异）。
- **处理规范**：查看 `/tmp/e2_probe_dump.json`，核验实车返回的原始 JSON 数据；当前 `e2_bridge_node` 与 `e2_api_client` 已全面兼容 Base64 栅格编码、二维数值矩阵以及嵌套的 `path`/`points`/JSON 字符串形式的路径响应；若发现实车返回了新字段，请将 dump 文件提取后补充适配解析逻辑。

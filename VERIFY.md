# Jetson 上机验证手册

陇参卫士除草机器人 ROS 软件，在 Jetson Nano（Ubuntu 18.04 + ROS Melodic）上的编译与逐节点验证步骤。本机（Windows）只用于编辑，所有编译、测试、运行均在 Jetson 上执行。

按 `plan.md` 第十一节开发顺序组织，可逐项核对。

---

## 0. 环境准备

```bash
# ROS 环境
source /opt/ros/melodic/setup.bash

# 工作区根目录
cd ~/Jetson_ws        # 或实际工作区路径
```

确认以下系统依赖已安装（plan 未要求新增第三方库，以下均为节点已用到的系统包）：

```bash
# 编解码、TCP、序列化
sudo apt install libboost-system-dev protobuf-compiler libprotobuf-dev

# 视觉与推理（plant_detection 需要，Jetson 上 JetPack 已自带 CUDA/TensorRT）
sudo apt install ros-melodic-cv-bridge ros-melodic-image-transport
# TensorRT (nvinfer / nvinfer_plugin) 随 JetPack 提供，路径 /usr/lib/aarch64-linux-gnu

# 测试
sudo apt install ros-melodic-rostest
```

OrbbecSDK_ROS1 子模块需已拉取：

```bash
ls src/OrbbecSDK_ROS1/orbbec_camera/launch/astra_pro_plus.launch    # 应存在
```

模型文件需放入 `src/weed_bringup/models/`：

```text
plant_detector.onnx       # 训练导出
plant_detector.engine     # 在本台 Jetson 上由 onnx 生成，与 TensorRT 版本绑定
classes.txt               # 已存在：crop / weed
```

> `.engine` 必须在目标 Jetson 上生成，不能跨机拷贝。

---

## 1. 编译

```bash
cd ~/Jetson_ws
catkin_make
source devel/setup.bash
```

成功标准：无报错，`devel/lib/` 下生成 8 个节点可执行文件。

```bash
ls devel/lib/plant_detection/plant_detection_node \
   devel/lib/plant_tracking/plant_tracking_node \
   devel/lib/plant_depth/plant_depth_node \
   devel/lib/weed_decision/weed_decision_node \
   devel/lib/weed_monitor/weed_monitor_node \
   devel/lib/chassis_bridge/chassis_bridge_node \
   devel/lib/serial_bridge/serial_bridge_node \
   devel/lib/weed_tf/weed_tf_node
```

若 `plant_detection` 因 TensorRT 头文件路径报错，确认 `NvInfer.h` 位置：

```bash
find /usr -name NvInfer.h 2>/dev/null
```

`plant_detection/CMakeLists.txt` 的 `HINTS` 已含 `/usr/include/aarch64-linux-gnu` 与 `/usr/lib/aarch64-linux-gnu`，多数 JetPack 默认布局可直接命中。

---

## 2. 自动化测试

5 个不依赖硬件的节点带 rostest 集成测试。整体跑：

```bash
cd ~/Jetson_ws
catkin_make run_tests
catkin_test_results          # 汇总通过 / 失败
```

单独跑某个：

```bash
rostest plant_tracking tracking.test
rostest plant_depth    depth.test
rostest weed_decision  decision.test
rostest weed_tf        tf.test
rostest weed_monitor   monitor.test
```

覆盖范围：

| 节点 | 验证点 |
|---|---|
| plant_tracking | 同类高 IOU 复用 track_id，跨类不匹配，远框分配新 id |
| plant_depth | 均匀深度反投影坐标，0 深度判 invalid，track_id/class 透传 |
| weed_decision | 稳定 3 帧后发目标，串口离线不发，crop/越界忽略，完整握手回 IDLE，急停进入与解除 |
| weed_tf | 静态 TF camera→arm_base 可查且为 identity（默认外参） |
| weed_monitor | 始终发 workspace + status marker，有效点出 marker，active_target，状态文本反映输入 |

`plant_detection`、`serial_bridge`、`chassis_bridge` 依赖真实硬件 / 外部服务，不在自动化测试内，手动验证见下文。

---

## 3. 逐节点验证（对应 plan 开发顺序）

### 第 6 项 相机图像 / 深度 / CameraInfo

```bash
roslaunch weed_bringup camera.launch
```

另开终端：

```bash
rostopic hz /camera/color/image_raw      # 期望 ~30 Hz
rostopic hz /camera/depth/image_raw      # 期望 ~30 Hz
rostopic echo -n1 /camera/color/camera_info     # K 矩阵非零
rosrun image_view image_view image:=/camera/color/image_raw
```

确认 `depth_registration=true` 生效：深度与彩色对齐（同一像素 (u,v) 对应同一物点）。

### 第 7 项 TF 标定

```bash
roslaunch weed_bringup tf.launch
rosrun tf tf_echo camera_color_optical_frame arm_base
```

`config/tf_camera_to_arm.yaml` 当前平移 / 旋转均为 0（占位）。**现场必须实测填入**：`arm_base` 定义 x 前、y 左、z 下。改完重启 tf.launch。

### 第 8 项 plant_depth

需相机已启动。

```bash
roslaunch weed_bringup camera.launch          # 终端 A
rosrun plant_depth plant_depth_node           # 终端 B
# 手工灌一帧 track 触发计算
rostopic pub -1 /plant/tracks plant_msgs/PlantTrack \
  '{header: {frame_id: "camera_color_optical_frame"}, track_id: 1, class_name: "weed", xmin: 300, ymin: 200, xmax: 340, ymax: 300, state: 0}'
rostopic echo -n1 /plant/points               # 终端 C：depth_valid 与 point 是否合理
```

> 注意：`plant_depth_node` 从私有命名空间 `~target_v_ratio` 等读参数，而 `config/depth.yaml` 加载到全局 `/depth/`，二者不匹配，节点实际使用代码默认值（与 yaml 数值恰好一致：ratio 0.75 / roi 7 / min 0.05 / max 1.50）。若要现场调参，用 `_param:=value` 或把 yaml 键挂到节点私有空间。

### 第 9 项 serial_bridge（ESP32-S3）

真实串口：

```bash
ls -l /dev/ttyTHS1                            # 确认设备存在、有读写权限
roslaunch weed_bringup arm.launch
rostopic echo /serial/status                  # 期望 online
rostopic echo /arm/state                      # ESP32 回帧后出 ACCEPTED/BUSY/DONE...
```

无 ESP32 时用虚拟串口对拍：

```bash
sudo apt install socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0     # 输出两个 /dev/pts/N
# 把 serial.yaml 的 port 改成其中一个 pts，另一个用脚本收发模拟 ESP32
```

发目标后抓 Jetson→ESP32 的 TARGET 帧，确认格式 `AA55` + CMD + ID + X + Y + Z + ERR + SUM + `\n`。

> **校验位待核对**：`plan.md` 第六节示例帧 `AA5501002A00EBFFEE0040001C` 的 SUM=`1C` 用常见算法均复现不出（详见 `PLAN_MISMATCHES.md`）。当前实现用 CMD..ERR 累加和取低 8 位。上机时必须与 ESP32-S3 固件实际校验实现对齐，不一致则以固件为准修正 `serial_bridge_node.cpp` 的 `checksum`。

### 第 10 项 chassis_bridge（RTK 底盘）

`config/chassis.yaml` 的 host/port/账号/任务名均为占位值，**现场替换为真实值**。

```bash
roslaunch weed_bringup chassis.launch
rostopic echo /chassis/status                 # sys_state/emergency_state/... 持续刷新
rostopic echo /chassis/task_state             # 地图任务状态
```

验证顺序：TCP 连上 → 登录拿 session_id → 每 10s 心跳 → 查询地图/任务/系统/硬件状态 → 启动地图任务。可在底盘端日志确认收到 `CMD_FULL_MOVEMENT_TASK`。

### 第 11 项 plant_detection（TensorRT YOLO）

需相机 + `.engine` 模型就位。

```bash
roslaunch weed_bringup camera.launch          # 终端 A
roslaunch weed_bringup tf.launch              # 终端 B（detection 本身不需要，链路完整性用）
rosparam load src/weed_bringup/config/detector.yaml
rosrun plant_detection plant_detection_node   # 终端 C
rostopic echo /plant/detections               # crop/weed 框，置信度，xyxy
```

确认 letterbox 坐标映射正确：检测框应贴合 640×480 原图中的目标，不偏移、不缩放错位。

### 第 12 项 plant_tracking

```bash
rosrun plant_tracking plant_tracking_node
# 配合 plant_detection 运行，观察 track_id 在连续帧中保持稳定
rostopic echo /plant/tracks
```

### 第 13 项 weed_decision

```bash
roslaunch weed_bringup tf.launch              # 需要 camera->arm_base TF
rosparam load src/weed_bringup/config/arm_workspace.yaml
rosparam load src/weed_bringup/config/tracker.yaml
rosrun weed_decision weed_decision_node
rostopic echo /weed/state                     # IDLE -> SENT -> WAIT_ARM -> IDLE
rostopic echo /weed/target                    # 锁定的 active_target
```

行为核对（已由 decision.test 覆盖，现场复核）：

- 仅处理 `class_name=="weed"` 且 `depth_valid` 且落在 arm_workspace 内的点
- 连续 3 帧稳定后才发，选距窗口中心最近
- 必须先收到 `/serial/status == online` 才发目标
- `/arm/state` DONE 记入已处理、ERROR 记入失败且第一版不重试
- 底盘 `emergency_state!=0` 或机械臂 BUSY 时串口掉线 → EMERGENCY
- SENT 后约 2s 无 ACCEPTED：若目标仍是有效候选则用最新坐标重发，否则回 IDLE

### weed_monitor（plan 第三-10 可视化）

```bash
rosrun weed_monitor weed_monitor_node
rosrun rviz rviz
# RViz: Fixed Frame 设为 arm_base，Add -> MarkerArray，topic 选 /weed/markers
```

应看到：arm_workspace 半透明立方体、weed/crop 3D 点球 + track_id 文本、active_target 黄色圆柱、右上角状态文本（weed/arm/serial/chassis）。

---

## 4. 整机启动（完成标准）

模型、外参、底盘连接、串口全部就位后：

```bash
roslaunch weed_bringup bringup.launch
```

`bringup.launch` 依次拉起 camera → tf → perception（detection/tracking/depth/decision/monitor）→ chassis → arm。

整机验收：

```bash
rostopic list                                 # 上述全部 topic 在线
rostopic echo /weed/state                      # 随检测流转
rostopic hz /plant/detections /plant/tracks /plant/points
```

底盘按预设路径前进，视觉链路发现 weed、满足作业条件后向 ESP32-S3 发坐标，机械臂作业期间底盘不停车。

### 第 14 / 15 项

- 第 14 项 联调底盘预设路径与行进作业：需真实底盘 + 机械臂在场，按上面 chassis + arm + decision 链路联调。
- 第 15 项 田间测试与参数调整：现场实测，重点调 arm_workspace 边界、tf 外参、detector 阈值、depth ROI。

---

## 5. 待人工确认项（见 PLAN_MISMATCHES.md）

| 项 | 现状 | 现场动作 |
|---|---|---|
| TF 外参 | 全 0 占位 | 实测 camera→arm_base 平移旋转 |
| 底盘连接 | IP/端口/账号/任务名占位 | 填真实值 |
| 串口 SUM 校验 | 累加和取低 8 位，与 plan 示例帧不符 | 对齐 ESP32 固件 |
| `.engine` 模型 | 未提供 | 本机 Jetson 由 onnx 生成 |
| depth 参数命名空间 | yaml 在 `/depth/`，节点读私有空间 | 现场调参时挂到节点私有空间 |

# 陇参卫士 Jetson Nano 软件开发计划

## 项目目标

Jetson Nano 作为机器人主控，负责视觉感知、目标定位、作业决策、底盘通信以及 ESP32-S3 通信。

目标运行环境：

```text
Jetson Nano
Ubuntu 18.04
ROS Melodic
```

本机只用于编辑、整理资料和维护工程文件。编译、运行和测试在 Jetson Nano 上完成。

---

# 一、总体架构

```text
Astra Pro Plus
      │
      ▼
orbbec_camera
      │
      ▼
plant_detection
      │
      ▼
plant_tracking
      │
      ▼
plant_depth
      │
      ▼
weed_decision
   │        │
   │        │
   ▼        ▼
chassis_bridge  serial_bridge
   │        │
   ▼        ▼
RTK 底盘   ESP32-S3
```

底盘按预设路径自动导航。Jetson 通过视觉链路发现杂草，并在目标满足作业条件后向 ESP32-S3 发送机械臂目标坐标。

机械臂作业时底盘不停车。底盘前进动力参与除草杆拔除动作。

---

# 二、硬件与驱动

## 1. Astra Pro Plus

使用 Orbbec 官方 ROS1 驱动：

```text
https://github.com/orbbec/OrbbecSDK_ROS1
branch: main
package: orbbec_camera
launch: astra_pro_plus.launch
```

相机参数：

```text
color_width  = 640
color_height = 480
color_fps    = 30

depth_width  = 640
depth_height = 480
depth_fps    = 30

depth_registration = true
enable_colored_point_cloud = false
```

常用输出：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
```

## 2. RTK 四轮底盘

通信方式：

```text
TCP 长连接
4 字节大端 BodySize + Protobuf body
协议：muses.proto
```

Jetson 负责：

```text
登录
心跳
查询状态
启动底盘地图任务
读取定位、速度、急停和任务状态
必要时触发急停
```

田间正常作业使用底盘已有地图任务和预设路径导航。

## 3. ESP32-S3

通信方式：

```text
UART
115200 8N1
ASCII 十六进制行协议
```

ESP32-S3 负责：

```text
解析目标坐标
逆运动学计算
除草杆动作控制
动作状态反馈
```

---

# 三、ROS package 规划

```text
src/
├── OrbbecSDK_ROS1/
├── weed_robot/
├── plant_msgs/
├── plant_detection/
├── plant_tracking/
├── plant_depth/
├── weed_tf/
├── weed_decision/
├── chassis_bridge/
├── serial_bridge/
├── weed_monitor/
└── weed_bringup/
```

`weed_robot` 是顶层 metapackage，不承载节点逻辑。

## 1. plant_msgs

共享消息包。

```text
PlantDetection.msg
PlantDetectionArray.msg
PlantTrack.msg
PlantPoint.msg
```

模型类别：

```text
0 crop
1 weed
```

`crop` 是作物保护对象。`weed` 是候选作业目标。

## 2. plant_detection

负责：

```text
订阅 RGB 图像
执行 TensorRT YOLO 推理
输出 crop / weed 检测框
```

输入：

```text
/camera/color/image_raw
```

输出：

```text
/plant/detections
```

YOLO 输入：

```text
640x480 RGB
→ letterbox 到 640x640
→ 推理
→ bbox 映射回 640x480 原始图像坐标
```

## 3. plant_tracking

负责：

```text
为 crop / weed 分配 track_id
保持连续帧目标稳定
输出带 ID 的目标
```

输入：

```text
/plant/detections
```

输出：

```text
/plant/tracks
```

## 4. plant_depth

负责：

```text
根据 weed 目标检测框读取对齐深度
取视觉目标点附近 ROI 的有效深度中位数
输出相机坐标系下三维目标点
```

输入：

```text
/plant/tracks
/camera/depth/image_raw
/camera/color/camera_info
```

输出：

```text
/plant/points
```

深度取值：

```text
u = (xmin + xmax) / 2
v = ymin + (ymax - ymin) * 0.75
→ 以 (u, v) 为中心的 5x5 或 7x7 ROI
→ 去掉 0 / NaN / 超出范围的深度
→ median depth
```

## 5. weed_tf

负责：

```text
发布 camera_color_optical_frame 到 arm_base 的静态 TF
```

`arm_base` 坐标系：

```text
x：机器人前方
y：机器人左方
z：向下
```

外参来源：

```text
人工测量
现场测试点微调
```

## 6. weed_decision

负责：

```text
过滤 crop
选择 weed 目标
判断 depth_valid
转换到 arm_base
判断作业窗口
管理作业状态机
向 ESP32-S3 发送目标
读取 ESP32-S3 作业结果
读取底盘状态
```

只处理：

```text
class_name == "weed"
depth_valid == true
point in arm_workspace
```

目标选择：

```text
连续 3 帧稳定
距离作业窗口中心最近
每次只发送一个 active_target
TARGET 发出后锁定坐标
DONE 后记录为已处理目标
ERROR 后记录为失败目标，第一版不立即重试
```

## 7. chassis_bridge

负责：

```text
维护 TCP 连接
LoginRequest 登录
保存 session_id
每 10 秒发送 CMD_HEARTBEATS_UPDATE
启动底盘地图任务
查询 SystemState / HardwareState
发布底盘状态
```

协议来源：

```text
K:\四轮底盘\北斗导航控制器协议(1)\北斗导航控制器协议\muses.proto
```

## 8. serial_bridge

负责：

```text
维护 ESP32-S3 串口
编码 Jetson → ESP32-S3 命令
解析 ESP32-S3 → Jetson 状态
发布串口在线状态
```

## 9. weed_bringup

负责：

```text
集中保存 launch
集中保存 config
集中保存模型相对路径
提供整机启动入口
```

## 10. weed_monitor

负责：

```text
发布 RViz Marker / Overlay
显示 crop / weed 检测框
显示 track_id
显示 3D 目标点
显示 arm_workspace
显示 active_target
显示底盘状态
显示 ESP32-S3 状态
```

第一版不开发独立 GUI。

---

# 四、Topic

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info

/plant/detections
/plant/tracks
/plant/points

/weed/target
/weed/state

/chassis/status
/chassis/task_state

/serial/status
/arm/state
```

---

# 五、消息定义

## PlantDetection.msg

```text
std_msgs/Header header
string class_name
float32 confidence
int32 xmin
int32 ymin
int32 xmax
int32 ymax
```

## PlantDetectionArray.msg

```text
std_msgs/Header header
PlantDetection[] detections
```

## PlantTrack.msg

```text
std_msgs/Header header
uint32 track_id
string class_name
float32 confidence
int32 xmin
int32 ymin
int32 xmax
int32 ymax
uint8 state
```

状态值：

```text
0 CANDIDATE_2D
1 VALID_3D
2 SENT_TO_ARM
3 DONE
4 FAILED
```

## PlantPoint.msg

```text
std_msgs/Header header
uint32 track_id
string class_name
geometry_msgs/Point point
bool depth_valid
```

---

# 六、ESP32-S3 串口协议

帧格式：

```text
AA55 CMD ID X Y Z ERR SUM\n
```

字段：

```text
AA55  帧头
CMD   命令码，uint8
ID    目标 ID，uint16
X     x_mm，int16
Y     y_mm，int16
Z     z_mm，int16
ERR   错误码，uint8
SUM   校验，uint8
```

发送时去掉空格：

```text
AA5501002A00EBFFEE0040001C\n
```

命令映射：

```text
01 TARGET
02 HOME
03 STOP
04 RESET
05 PING

81 ACCEPTED
82 BUSY
83 DONE
84 ERROR
85 READY
86 PONG
```

坐标单位：

```text
mm
```

ROS 内部使用米，发送前转换：

```text
x_mm = round(x * 1000)
y_mm = round(y * 1000)
z_mm = round(z * 1000)
```

握手流程：

```text
TARGET
  ↓
ACCEPTED
  ↓
BUSY
  ↓
DONE
```

异常：

```text
CMD = 84 ERROR
ID  = target_id
ERR = error_code
```

第一版不重复发送同一个 TARGET。

ESP32 动作流程：

```text
ACCEPTED
→ 逆运动学
→ BUSY
→ 移动到 approach point
→ 移动到 insert point
→ 保持
→ 移动到 retract point
→ DONE
```

动作参数保存在 ESP32-S3 本地：

```text
approach_height_mm
insert_depth_mm
hold_ms
retract_height_mm
move_speed
insert_speed
```

---

# 七、底盘控制流程

启动流程：

```text
建立 TCP 连接
LoginRequest 登录
保存 session_id
启动心跳
查询地图和当前任务
启动底盘地图任务
持续读取 SystemState
```

正常作业：

```text
底盘按预设路径前进
Jetson 持续检测 crop / weed
weed_decision 筛选可作业 weed
ESP32-S3 执行除草动作
底盘不停车
```

Jetson 关注的底盘状态：

```text
sys_state
location_state
emergency_state
mc_state.v_x
mc_state.v_y
mc_state.w
movement_state
gnss_msg_state
fault_code
```

系统急停触发条件：

```text
底盘 emergency_state 已触发
底盘 fault_code != 0 且处于运动中
ESP32 串口离线且机械臂处于 BUSY
ESP32 返回严重 ERROR
用户手动触发 STOP
```

普通感知失败不触发急停，只停止发送新目标。

---

# 八、配置文件

配置集中放在：

```text
weed_bringup/config/
```

文件：

```text
camera.yaml
detector.yaml
tracker.yaml
depth.yaml
tf_camera_to_arm.yaml
arm_workspace.yaml
chassis.yaml
serial.yaml
```

## detector.yaml

```yaml
detector:
  engine_path: models/plant_detector.engine
  onnx_path: models/plant_detector.onnx
  classes_path: models/classes.txt
  input_width: 640
  input_height: 640
  confidence_threshold: 0.35
  nms_threshold: 0.45
```

模型路径以 `weed_bringup` 包为基准解析。

## arm_workspace.yaml

```yaml
arm_workspace:
  x_min: 0.10
  x_max: 0.45
  y_min: -0.20
  y_max: 0.20
  z_min: 0.00
  z_max: 0.25
```

## serial.yaml

```yaml
serial:
  port: /dev/ttyTHS1
  baudrate: 115200
  accepted_timeout_ms: 500
  pong_timeout_ms: 3000
  ping_interval_ms: 1000
```

---

# 九、模型与数据集

数据集：

```text
T:\CropWeeds-YOLO Dataset\dataset
```

类别：

```text
0 crop
1 weed
```

本地结构：

```text
images/train
images/val
images/test
labels/train
labels/val
labels/test
classes.txt
```

部署流程：

```text
训练 YOLO
导出 ONNX
Jetson 上生成 TensorRT engine
C++ 节点加载 engine 推理
```

模型目录：

```text
weed_bringup/models/
├── plant_detector.onnx
├── plant_detector.engine
└── classes.txt
```

`.engine` 与 Jetson / TensorRT 版本强绑定，在目标机生成。

---

# 十、启动文件

```text
weed_bringup/
└── launch/
    ├── camera.launch
    ├── perception.launch
    ├── chassis.launch
    ├── arm.launch
    ├── tf.launch
    └── bringup.launch
```

入口：

```bash
roslaunch weed_bringup bringup.launch
```

---

# 十一、开发顺序

1. 创建 ROS workspace 和 package 骨架
2. 加入 OrbbecSDK_ROS1 main 分支
3. 编写 weed_robot metapackage
4. 编写 plant_msgs
5. 编写 weed_bringup 的 config 与 launch
6. 跑通 Astra Pro Plus 图像、深度和 CameraInfo
7. 标定 camera_color_optical_frame 到 arm_base
8. 完成 plant_depth 的 RGB-D 三维点计算
9. 完成 serial_bridge 与 ESP32-S3 协议
10. 完成 chassis_bridge 登录、心跳、状态读取
11. 完成 plant_detection 的 TensorRT 推理
12. 完成 plant_tracking
13. 完成 weed_decision 状态机
14. 联调底盘预设路径与行进作业
15. 田间测试与参数调整

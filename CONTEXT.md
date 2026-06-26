# 陇参卫士

陇参卫士是运行在 Jetson Nano 上的田间机器人主控软件上下文，用于组织视觉感知、目标定位、作业决策以及外设通信。

## Language

**目标运行环境**:
Jetson Nano + Ubuntu 18.04 + ROS Melodic。
_Avoid_: 本机 Noetic 缓存环境、Windows 开发环境

**本机开发环境**:
用于编辑与文件管理的开发环境，不作为编译、运行和测试目标。
_Avoid_: 目标运行环境

**weed_robot**:
陇参卫士 ROS 软件的顶层 metapackage。
_Avoid_: 单体功能包

**功能包**:
承载一个清晰职责的独立 catkin package。
_Avoid_: 模块目录、脚本目录

**Astra Pro Plus**:
陇参卫士使用的 Orbbec RGB-D 相机型号。
_Avoid_: Astra、Astra Pro、Astra+

**OrbbecSDK_ROS1 main**:
陇参卫士采用的 Astra Pro Plus ROS1 驱动来源。
_Avoid_: OrbbecSDK_ROS1 v2-main、OrbbecSDK_ROS2、pyorbbecsdk

**深度彩色对齐**:
Astra Pro Plus 的深度图对齐到彩色图，用于从 RGB 检测框读取深度。
_Avoid_: 未对齐深度取点

**2D 候选目标**:
YOLO 已在 RGB 图像中发现但尚未得到有效深度的杂草目标。
_Avoid_: 有效作业目标

**目标深度值**:
视觉目标点附近有效深度像素的中位数。
_Avoid_: 单个中心像素深度

**视觉目标点**:
检测框水平居中、垂直靠下的像素点，用于更接近杂草根部位置。
_Avoid_: 检测框几何中心点

**active_target**:
当前唯一发送给 ESP32-S3 并等待作业结果的杂草目标。
_Avoid_: 同时发送多个 TARGET

**稳定目标**:
连续 3 帧满足 weed、depth_valid 和作业窗口条件的目标。
_Avoid_: 单帧作业目标

**锁定目标**:
发送给 ESP32-S3 后不再更新坐标的 active_target。
_Avoid_: 执行中持续改写目标坐标

**已处理目标**:
ESP32-S3 返回 DONE 后记录的杂草目标，用于避免重复作业。
_Avoid_: 同一株杂草重复发送 TARGET

**失败目标**:
ESP32-S3 返回 ERROR 后记录的杂草目标，第一版不立即重试。
_Avoid_: 机械错误后反复发送同一目标

**系统急停**:
由底盘故障、机械臂忙时串口离线、严重 ESP32 错误或用户 STOP 触发的安全状态。
_Avoid_: 普通感知失败触发急停

**weed_monitor**:
第一版基于 RViz Marker 和 Overlay 展示视觉、目标、TF、底盘和 ESP32 状态。
_Avoid_: 第一版开发独立 GUI

**相机机械臂外参**:
从相机光学坐标系到机械臂基坐标系的静态 TF 变换。
_Avoid_: 写死在深度节点里的坐标偏移

**外参人工标定**:
通过机械测量和现场测试点微调得到相机到机械臂的 TF 参数。
_Avoid_: 第一版引入复杂标定流程

**作业窗口**:
机械臂在 `arm_base` 坐标系下允许接收目标点的三维范围。
_Avoid_: 图像作业区域、深度有效区域

**机械臂坐标系**:
ESP32-S3 逆运动学使用的 `arm_base` 坐标系，`x` 向前，`y` 向左，`z` 向下。
_Avoid_: 默认 z 向上的机器人底盘坐标

**ESP32 目标坐标**:
Jetson 通过 UART 发送给 ESP32-S3 的 `arm_base` 坐标，单位为毫米整数。
_Avoid_: 米制浮点字符串

**作业握手**:
Jetson 与 ESP32-S3 围绕同一个 `target_id` 完成接收、执行和结束确认。
_Avoid_: 无 ID 的 DONE

**ESP32 十六进制协议**:
Jetson 与 ESP32-S3 使用 ASCII 十六进制行协议传递命令、目标坐标和执行状态。
_Avoid_: 自然语言命令字符串

**ESP32 串口链路**:
Jetson 与 ESP32-S3 使用 115200 8N1 串口通信，并通过 PING/PONG 监测在线状态。
_Avoid_: 无超时状态的串口发送

**ESP32 作业状态机**:
ESP32-S3 收到 `TARGET` 后完成接收、逆运动学、移动、下插、保持、回收和结束反馈。
_Avoid_: 收到坐标后无状态反馈

**ESP32 动作参数**:
ESP32-S3 本地保存的接近高度、下插深度、保持时间、回收高度和动作速度。
_Avoid_: Jetson 每次发送完整动作过程

**ESP32 动作点**:
ESP32-S3 根据 `TARGET` 和动作参数生成的 approach、insert、retract 三个机械臂目标点。
_Avoid_: Jetson 发送完整轨迹

**行进作业**:
底盘保持前进，由底盘牵引力配合除草杆完成拔除。
_Avoid_: 停车作业

**底盘控制链路**:
Jetson 通过 TCP 长连接和 muses Protobuf 协议控制四轮 RTK 底盘。
_Avoid_: 底盘自主匀速

**预设路径导航**:
底盘按已有地图任务或站点任务自动前进。
_Avoid_: 田间作业全程手动速度控制

**底盘地图任务**:
底盘配套 APP 或控制器地图系统中已有的路径、站点和移动任务。
_Avoid_: Jetson 第一版自行绘制 RTK 路径

**weed_bringup**:
集中管理相机、感知、底盘、机械臂、TF 和整机启动的 launch 包。
_Avoid_: 各功能包分散保存总启动文件

**bringup 参数集**:
`weed_bringup/config` 下集中保存现场可调参数的 YAML 文件集合。
_Avoid_: 参数散写在代码里

**相对模型路径**:
模型路径以 `weed_bringup` 包为基准解析，不在配置里写绝对路径。
_Avoid_: `/home/...` 绝对模型路径

**plant_msgs**:
陇参卫士 ROS 节点之间共享的作物与杂草检测、跟踪和三维目标消息包。
_Avoid_: 各节点自定义私有消息

**混合语言节点**:
按模块特性选择 Python 或 C++ 实现的 ROS 节点集合。
_Avoid_: 全部强行使用同一种语言

**部署运行链路**:
Jetson Nano 上长期运行的 ROS 节点链路，优先使用 C++ 实现。
_Avoid_: 依赖 Python 3 版 ROS 图像桥接

**YOLO 输入图像**:
由 640x480 RGB 图像 letterbox 到 640x640 后送入模型。
_Avoid_: 未记录缩放关系的检测框

**作物杂草类别**:
YOLO 数据集中的两类目标：`crop` 和 `weed`。
_Avoid_: 单类别 weed

## Relationships

- **目标运行环境**承载 ROS 节点的编译、运行和测试
- **本机开发环境**只承载项目文件编辑与计划整理
- **weed_robot**包含多个**功能包**
- **weed_bringup**保存整体启动入口和 launch 配置
- **Astra Pro Plus**向视觉链路提供 RGB 图像、Depth 图像和相机参数
- **OrbbecSDK_ROS1 main**通过 `astra_pro_plus.launch` 启动 **Astra Pro Plus**
- **深度彩色对齐**由 `depth_registration:=true` 启用
- **2D 候选目标**只有得到有效对齐深度后才进入作业决策
- **视觉目标点**由 `plant_depth` 从 weed 检测框中选取
- **目标深度值**由 `plant_depth` 在 **视觉目标点** 附近从对齐深度图中计算
- **active_target**由 `weed_decision` 从作业窗口内的 weed 目标中选择
- **稳定目标**才能成为 **active_target**
- **active_target**发送后成为 **锁定目标**
- **锁定目标**完成后进入 **已处理目标**
- **锁定目标**失败后进入 **失败目标**
- **系统急停**优先级高于作业状态机和底盘任务
- **weed_monitor**用于联调观察系统状态
- **相机机械臂外参**由 `weed_tf` 发布，供目标点转换到 `arm_base`
- **外参人工标定**产出 **相机机械臂外参** 参数
- **作业窗口**过滤可发送给 ESP32-S3 的目标点
- **机械臂坐标系**是发送 ESP32-S3 目标坐标的解释基准
- **ESP32 目标坐标**由 `weed_decision` 从米制 ROS 点转换得到
- **作业握手**决定 `weed_decision` 何时离开 `WAIT_ARM`
- **ESP32 十六进制协议**承载 **ESP32 目标坐标** 和 **作业握手**
- **ESP32 串口链路**在异常时阻止发送新的作业目标
- **ESP32 作业状态机**决定 `DONE` 或 `ERROR` 的返回时机
- **ESP32 动作参数**决定同一个目标点如何完成下插和回收
- **ESP32 动作点**由 ESP32-S3 在本地计算
- **行进作业**要求 `WAIT_ARM` 期间底盘继续前进
- **底盘控制链路**由 `chassis_bridge` 维护登录、心跳、状态查询和任务启动
- **预设路径导航**是田间正常作业时的底盘行走方式
- **底盘地图任务**由 `chassis_bridge` 查询并启动
- **weed_bringup**提供 `bringup.launch` 作为整机启动入口
- **bringup 参数集**由 `weed_bringup` 加载并传给各节点
- **相对模型路径**用于定位 TensorRT engine、ONNX 和类别文件
- **plant_msgs**定义 `PlantDetection`、`PlantDetectionArray`、`PlantTrack` 和 `PlantPoint`
- **混合语言节点**中 ROS 运行节点优先 C++，训练、导出和工具脚本使用 Python
- **部署运行链路**不依赖 Python 3 版 `cv_bridge`
- **YOLO 输入图像**的检测框需要映射回 640x480 原始 RGB 坐标
- **作物杂草类别**中只有 `weed` 能进入作业决策

## Example dialogue

> **Dev:** “这个 ROS package 要按哪个发行版写？”
> **Domain expert:** “按 **目标运行环境** 写，也就是 Jetson Nano + Ubuntu 18.04 + ROS Melodic。”
> **Dev:** “launch 文件放在顶层目录吗？”
> **Domain expert:** “放进 **weed_bringup**，它负责组织系统启动。”
> **Dev:** “相机型号按 Astra 写可以吗？”
> **Domain expert:** “写 **Astra Pro Plus**，避免和 Astra、Astra Pro、Astra+ 混用。”
> **Dev:** “用 OrbbecSDK_ROS1 的哪个分支？”
> **Domain expert:** “用 **OrbbecSDK_ROS1 main**，Astra Pro Plus 在 v2-main 里不支持。”
> **Dev:** “RGB 检测框可以用同一组像素坐标读深度吗？”
> **Domain expert:** “可以，前提是启用 **深度彩色对齐** 并读到有效深度。”
> **Dev:** “YOLO 看到了杂草但还没有深度，可以发给机械臂吗？”
> **Domain expert:** “不可以，它只是 **2D 候选目标**。”
> **Dev:** “深度用检测框中心的一个像素吗？”
> **Domain expert:** “不用，取 **视觉目标点** 附近有效深度的中位数作为 **目标深度值**。”
> **Dev:** “TARGET 用检测框中心吗？”
> **Domain expert:** “不用，使用更靠近根部的 **视觉目标点**。”
> **Dev:** “多个杂草目标可以同时发给 ESP32 吗？”
> **Domain expert:** “不可以，每次只有一个 **active_target**。”
> **Dev:** “单帧看到 weed 就能发 TARGET 吗？”
> **Domain expert:** “不能，必须先成为 **稳定目标**。”
> **Dev:** “TARGET 发出后还持续改坐标吗？”
> **Domain expert:** “不改，进入 **锁定目标**。”
> **Dev:** “DONE 后同一株草还会再次作业吗？”
> **Domain expert:** “不会，记录为 **已处理目标**。”
> **Dev:** “ERROR 后立即重试吗？”
> **Domain expert:** “不立即重试，记录为 **失败目标**。”
> **Dev:** “YOLO 没看到杂草就急停吗？”
> **Domain expert:** “不急停，只有安全相关异常进入 **系统急停**。”
> **Dev:** “第一版要单独写 GUI 吗？”
> **Domain expert:** “不用，使用 **weed_monitor** 配合 RViz。”
> **Dev:** “相机到机械臂的偏移写在哪里？”
> **Domain expert:** “写成 **相机机械臂外参**，由 TF 发布。”
> **Dev:** “第一版用标定板自动标定吗？”
> **Domain expert:** “不用，采用 **外参人工标定**。”
> **Dev:** “有三维坐标就能发给机械臂吗？”
> **Domain expert:** “还要落在 **作业窗口** 内。”
> **Dev:** “`arm_base` 的 z 轴向上吗？”
> **Domain expert:** “不向上，**机械臂坐标系**里 z 向下。”
> **Dev:** “串口发米制浮点还是毫米整数？”
> **Domain expert:** “发 **ESP32 目标坐标**，也就是毫米整数。”
> **Dev:** “ESP32 回一个 DONE 就行吗？”
> **Domain expert:** “DONE 要带 `target_id`，属于一次 **作业握手**。”
> **Dev:** “串口命令用文本单词吗？”
> **Domain expert:** “不用，采用 **ESP32 十六进制协议**。”
> **Dev:** “ESP32 离线时还继续发目标吗？”
> **Domain expert:** “不发，由 **ESP32 串口链路** 状态控制。”
> **Dev:** “ESP32 收到 TARGET 后马上 DONE 吗？”
> **Domain expert:** “不马上 DONE，要执行 **ESP32 作业状态机**。”
> **Dev:** “Jetson 每次都发下插深度和保持时间吗？”
> **Domain expert:** “不发，这些属于 **ESP32 动作参数**。”
> **Dev:** “approach、insert、retract 由 Jetson 算吗？”
> **Domain expert:** “不算，这些是 **ESP32 动作点**。”
> **Dev:** “机械臂作业时底盘停下来吗？”
> **Domain expert:** “不停车，采用 **行进作业**。”
> **Dev:** “底盘匀速由谁控制？”
> **Domain expert:** “由 Jetson 通过 **底盘控制链路** 控制。”
> **Dev:** “田间正常作业靠手动速度一直走吗？”
> **Domain expert:** “田间正常作业采用 **预设路径导航**。”
> **Dev:** “Jetson 第一版自己画 RTK 路径吗？”
> **Domain expert:** “不画，使用 **底盘地图任务**。”
> **Dev:** “整机启动文件散落在各个包里吗？”
> **Domain expert:** “不散落，由 **weed_bringup** 集中管理。”
> **Dev:** “现场调参改代码吗？”
> **Domain expert:** “不改代码，改 **bringup 参数集**。”
> **Dev:** “模型配置里写 `/home/jetson/...` 吗？”
> **Domain expert:** “不写，使用 **相对模型路径**。”
> **Dev:** “节点之间各自传 Python dict 可以吗？”
> **Domain expert:** “不可以，使用 **plant_msgs**。”
> **Dev:** “所有节点都用 Python 写吗？”
> **Domain expert:** “不强制，采用 **混合语言节点**。”
> **Dev:** “Jetson 上长期运行时依赖 Python 3 ROS 图像桥吗？”
> **Domain expert:** “不依赖，**部署运行链路**优先用 C++。”
> **Dev:** “YOLO 输出框可以按 640x640 坐标发出去吗？”
> **Domain expert:** “不可以，要从 **YOLO 输入图像**映射回原始 RGB 坐标。”
> **Dev:** “模型只检测 weed 吗？”
> **Domain expert:** “数据集包含 **作物杂草类别**，模型检测 `crop` 和 `weed`。”

## Flagged ambiguities

- 当前工作区存在 Noetic/Focal 的 `build/`、`devel/` 生成产物；已定为历史缓存，不参与项目设计判断。
- `weed_robot` 已定为顶层 metapackage，不承载具体节点逻辑。

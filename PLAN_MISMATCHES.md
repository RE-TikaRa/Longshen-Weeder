# 与 plan.md 不完全一致的自定项

用户已授权实现时自行决定。以下内容用于后续人工查看。

## depth.yaml

- `roi_size` 选为 `7`。
- `min_depth` 设为 `0.05`。
- `max_depth` 设为 `1.50`。

plan.md 写的是 `5x5 或 7x7 ROI`，没有指定唯一值。

## camera.yaml

`camera.launch` 通过 `<arg>` 直接向 `orbbec_camera` 的 `astra_pro_plus.launch` 传分辨率/帧率/对齐参数，不加载 `camera.yaml`。`camera.yaml` 的取值与 launch 中的 arg 完全一致，作为参数留档存在，当前不被任何 launch 注入参数服务器。两者数值一致，无冲突。

## tf_camera_to_arm.yaml

- `parent_frame` 设为 `camera_color_optical_frame`。
- `child_frame` 设为 `arm_base`。
- 平移和 RPY 旋转暂设为 `0.0`。

plan.md 写的是人工测量与现场测试点微调，没有提供实际外参。

## chassis.yaml

- `host` 设为 `192.168.1.100`。
- `port` 设为 `9000`。
- `username` 设为 `admin`。
- `password_md5` 设为 `21232f297a57a5a743894a0e4a801fc3`。
- `map_name` 设为 `default`。
- `task_name` 设为 `field_row_001`。

plan.md 和底盘资料没有给出现场 IP、端口和任务标识。

## launch 节点可执行文件名

- `plant_detection_node`
- `plant_tracking_node`
- `plant_depth_node`
- `weed_decision_node`
- `weed_monitor_node`
- `chassis_bridge_node`
- `serial_bridge_node`
- `weed_tf_node`

plan.md 没有指定节点可执行文件名。

## 开发顺序

- 第 6 项相机实机验证暂未执行。
- 已继续创建 `weed_tf_node` 和 `plant_depth_node`。

plan.md 要求先跑通 Astra Pro Plus 图像、深度和 CameraInfo。本机没有 Jetson Nano、ROS Melodic 和 Astra Pro Plus 运行环境。

## serial_bridge

- `/weed/target` 使用 `plant_msgs/PlantPoint`。
- `/serial/status` 使用 `std_msgs/String`。
- `/arm/state` 使用 `std_msgs/String`。
- `SUM` 使用 `CMD` 到 `ERR` 字节求和后取低 8 位。

plan.md 指定了 topic 名称和帧字段，没有指定 ROS topic 消息类型和校验算法。

plan.md 第六节示例帧 `AA5501002A00EBFFEE0040001C` 的校验字节是 `1C`。按字段拆分为 `CMD=01 ID=002A X=00EB Y=FFEE Z=0040 ERR=00`，用累加和、含帧头累加和、XOR、补码、按位取反等常见算法都无法复现 `1C`（累加和取低 8 位得 `43`）。该示例帧的 SUM 看起来是文档占位值。当前实现采用累加和取低 8 位，上机联调时必须以 ESP32-S3 固件实际校验实现为准。

`serial.yaml` 的 `accepted_timeout_ms` 未被 serial_bridge 读取。该超时语义（发 TARGET 后等 ACCEPTED 超时重发）已在 weed_decision 用 `/arm_workspace/accepted_timeout_s` 实现，serial_bridge 不重复实现。`pong_timeout_ms` 与 `ping_interval_ms` 已在 serial_bridge 实现：周期发 PING，距上次收帧超过 `pong_timeout_ms` 判离线并重连。

`port` 取 `/dev/ttyUSB0`，plan.md serial.yaml 写的是 `/dev/ttyTHS1`。实接方案改用 ESP32-S3 板载 COM 口（CH343P USB 转串口）经一根 Type-C 线连 Jetson，Jetson 侧枚举为 `/dev/ttyUSB0`，不走 Jetson 40-pin 的 ttyTHS1。COM 口桥接 ESP32-S3 的 UART0（GPIO43/44），ESP 固件 console 已从 UART0 改走 USB-Serial-JTAG 让出该口（详见 ESP 项目 `esp32-plan.md` 第二节与 `sdkconfig.defaults`）。

## plant_detection 输出张量布局

- `decode` 按 `[num_boxes][4 + num_classes]` 行优先布局读取，无 objectness 通道（YOLOv8 类别风格）。
- `num_boxes` 由 `output_size_ / (4 + num_classes)` 推算。

plan.md 只规定 letterbox 640→640 与 bbox 映射回原图，没有规定输出张量布局。YOLOv8 ONNX 默认导出形状为 `[1, 4+nc, 8400]`（属性维在前），与当前行优先假设相反，除非导出时已做 transpose。上机用真实 `.engine` 验证检测框是否贴合目标；若框错位或全空，需按导出实际布局调整 `decode` 的索引方式。


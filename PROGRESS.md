# 开发进度

依据：`plan.md` 第十一节。

## 当前状态

| 序号 | 阶段 | 状态 |
| --- | --- | --- |
| 1 | 创建 ROS workspace 和 package 骨架 | 已创建 |
| 2 | 加入 OrbbecSDK_ROS1 main 分支 | 已加入 |
| 3 | 编写 weed_robot metapackage | 已创建 |
| 4 | 编写 plant_msgs | 已创建 |
| 5 | 编写 weed_bringup 的 config 与 launch | 已创建 |
| 6 | 跑通 Astra Pro Plus 图像、深度和 CameraInfo | 待 Jetson 验证 |
| 7 | 标定 camera_color_optical_frame 到 arm_base | 使用自定初值并已创建 TF 节点 |
| 8 | 完成 plant_depth 的 RGB-D 三维点计算 | 已创建初版节点 |
| 9 | 完成 serial_bridge 与 ESP32-S3 协议 | 已创建初版节点 |
| 10 | 完成 chassis_bridge 登录、心跳、状态读取 | 已创建初版节点 |
| 11 | 完成 plant_detection 的 TensorRT 推理 | 已创建初版节点 |
| 12 | 完成 plant_tracking | 已创建初版节点 |
| 13 | 完成 weed_decision 状态机 | 已创建初版节点 |
| 14 | 联调底盘预设路径与行进作业 | 待 Jetson 验证 |
| 15 | 田间测试与参数调整 | 待 Jetson 验证 |

## 验证状态

- 当前 Windows 工作区没有 ROS Melodic 与 `catkin_make`。
- package 编译验证需要在 Jetson Nano 的 Ubuntu 18.04 + ROS Melodic 环境执行。

## 当前说明

- 第 6 项需要 Jetson Nano、Ubuntu 18.04、ROS Melodic 和 Astra Pro Plus。
- 8 个节点初版已创建，5 个不依赖硬件的节点带 rostest 集成测试。
- 上机编译与逐节点验证步骤见 `VERIFY.md`。
- 自定项与待上机核对项记录在 `PLAN_MISMATCHES.md`。

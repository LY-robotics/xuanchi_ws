# 修改日志

本文件记录每次对 `xc_motor_hardware` 的代码、配置和文档修改。日期使用中国标准时间。

## 2026-11-02

### 复核追加

- 根据正式 PDF 修正上一版实现中 A4、AE、B0 的字段定义：A4/AE 温度均为无符号字节，B0 为 DLC=7 且力矩常数为 4 字节小端单精度浮点数。
- 更新 B0 单元测试、vcan 模拟节点、协议文档和状态诊断结构。
- 新增 `SINGLE_ARM_QUICK_TEST_CN.md`，说明从物理接口识别、A3/A2/AE/B0 只读、当前 Count 的 C2 保持命令到五轴 ros2_control 的最短安全路径。
- 新增右臂专用 controller_manager 和 MoveIt controller 示例，明确 ros2_control XML、YAML、控制器 joints、MoveIt controller joints 必须对应；未使用控制器不要启动。
- 新增 `examples/right_arm_can_probe.sh`，用于只读发送 A3/A2/AE/B0，不发送 C2、B1 或 CF。
- SANPO SPINE V8 网页核对结果：Linux 使用 SocketCAN `canX`，透传模式直接转发标准 CAN 帧，不添加 ST/ET 报头；接口编号取决于 USB 枚举，首次需要逐接口确认并配置 1 Mbps。

### 协议依据

- 读取并核对《自定义CAN通信协议_V3.07b0.pdf》16 页内容。
- 核对 SANPO SPINE V8 SocketCAN 文档：USB 转 CAN 在 Linux 下显示为 `canX` 网络接口；SocketCAN 透传直接发送标准 CAN 帧，不添加 ST/ET 协议模式报头；CAN 接口需要系统配置为 1 Mbps。

### 代码修改

- 修正 A4：温度改为无符号字节，Q 轴电流为小端有符号 16 位，速度为小端有符号 16 位，单圈角度为小端无符号 16 位。
- 修正 AE：母线电压和母线电流为小端无符号 16 位，温度为无符号字节，运行模式和故障码按字节解析。
- 修正 B0：响应 DLC 改为 7；力矩常数改为小端 IEEE-754 单精度浮点数；协议减速比保留为诊断字段，不与 YAML 机械减速比相乘。
- 更新 B0 虚拟电机响应和协议单元测试。
- 保留 `motor_shaft`/`joint_output` 两种位置计数参考；位置反馈使用多圈 Count。

### 文档和测试修改

- 更新 `PROTOCOL_CN.md`、`README_CN.md`，记录正式 PDF 的字段布局和 SocketCAN 透传边界。
- 新增本修改日志。
- 单臂快速验证流程补充到 `HARDWARE_COMMISSIONING_CN.md` 和 `BUILD_AND_TEST_CN.md`。
- 当前 Windows 环境仍无 ROS 2、colcon 和 C++ 编译器，未声称 C++/ROS 构建通过。

### 待现场确认

- SANPO SPINE 的 `canX` 与板上物理 CAN 接口的对应关系；官方文档明确编号取决于 USB 枚举，不能仅凭 `can0` 判断物理端口。
- 右臂电机地址 34、57、14、32、21 是否与现场接线一一对应。
- 方向、机械减速比与 `encoder_zero_count`，以及电机是否已经处于可接受的输出状态。
- 真实设备的反馈周期、总线负载、急停和故障恢复策略。

## 2026-11-01

- 建立通用 `SanpoCanSystem` ros2_control 插件、协议层、SocketCAN 层、关节换算层和 YAML 配置。
- 增加右臂、左臂、上半身模板、协议/换算测试、vcan 模拟节点和中文开发文档。
- 增加配置校验、非法 CAN 帧拒绝、线程 RAII 清理、反馈超时、故障拒绝运动命令和可选停机策略。

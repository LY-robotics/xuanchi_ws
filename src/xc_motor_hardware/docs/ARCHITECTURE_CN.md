# 架构说明

## 分层

1. `motor_protocol.*` 只处理 CAN 帧编码、解码、小端序、长度和地址校验，不依赖 ROS。
2. `socketcan_transport.*` 只处理 Linux RAW CAN socket、接口绑定、过滤、非阻塞接收和 RAII 关闭；Windows 编译时提供失败桩，便于协议核心测试。
3. `joint_conversion.*` 只处理关节弧度、编码器 Count、RPM 和方向/减速比/零位换算。
4. `motor_config.*` 负责 YAML 读取及关节、ID、方向、减速比和接口匹配检查。
5. `sanpo_can_system.*` 是 ROS 2 生命周期适配层，导出接口并协调上面各层。

## 数据流

`write()` 检查控制器的 rad 命令，经 `JointConversion::joint_to_count()` 取整后交给 `MotorProtocol::make_c2_position_command()`，再由 SocketCAN 发送。接收线程持续收帧，解析 A2/A3/C2/AE/B0 并更新互斥保护的缓存；`read()` 只检查缓存，不等待 CAN 应答。

同一个 `SystemInterface` 可以导出 5、10 或 12 个关节接口。控制器按关节集合申请接口；MoveIt 规划组只是规划语义分组，不要求每个规划组对应一个硬件插件。

## 生命周期

`on_init()` 只解析 HardwareInfo 和 YAML，不运动；`on_configure()` 打开 socket、安装 ID 过滤并启动接收线程；`on_activate()` 请求每个电机 A3，收到全部初始位置后把命令同步到反馈；`read()` 周期请求和检查超时；`write()` 非阻塞发送 C2；`on_deactivate()` 按配置决定是否 CF；`on_cleanup()` 停线程、关 socket、清缓存。

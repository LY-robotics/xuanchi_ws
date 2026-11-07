# 代码阅读指南

- `motor_protocol.hpp/cpp`：`CanFrame` 是跨平台帧；`make_request()` 统一生成 `0x100 | id` 请求；`make_c2_position_command()` 负责 int32 小端编码；各 `parse_*()` 先检查标准帧、响应 ID、DLC、命令字再读字段。A4/AE 的温度按无符号字节解析，B0 的 Data[2..5] 按小端 IEEE-754 单精度解析；`decode_faults()` 把位掩码转换为中文故障名。
- `joint_conversion.hpp/cpp`：所有数学换算集中于三个函数。输入会检查有限浮点数、正减速比、方向和 int32 范围，避免溢出或 NaN 进入控制周期。
- `motor_config.cpp`：`load_hardware_config()` 读取 YAML；`HardwareConfig::validate()` 检查数量、名字、ID、方向、减速比。配置错误应在 `on_init()` 阶段暴露。
- `socketcan_transport.cpp`：Linux 下创建 RAW socket、绑定接口、设置 O_NONBLOCK；`receive()` 用 select 加超时，永不要求控制周期等待；Windows 返回明确“不支持 SocketCAN”。
- `SanpoCanSystem::on_init()`：读取 `config_file`，确认接口类型和关节匹配；`export_*_interfaces()` 将缓存地址交给 ros2_control；生命周期函数只负责资源和状态切换。
- `receiver_loop()`：唯一的 CAN 接收线程，解析后更新时间戳；`read()` 只做周期请求和缓存超时/故障检查；`write()` 只做命令验证、Count 转换和 C2 发送。

需要修改协议字段时改 `motor_protocol.cpp`；需要修改公式改 `joint_conversion.cpp` 并同步测试；需要增加配置参数改 `motor_config.*`；需要调整生命周期或控制器接口改 `sanpo_can_system.*`。

## 按调用顺序阅读

1. `on_init()` 调用基类初始化，然后检查每个 HardwareInfo 关节是否有唯一的 position 命令、position 状态和 velocity 状态。它读取 hardware 参数中的 `config_file`，由 `load_hardware_config()` 解析 YAML，最后调用 `HardwareConfig::validate()`。此时没有 socket，也不会发送电机命令。
2. `export_state_interfaces()` 和 `export_command_interfaces()` 将缓存成员的地址交给 ros2_control；因此缓存的生命周期必须覆盖控制器运行期，清理后不能继续访问接口。
3. `on_configure()` 创建 `SocketCanTransport`，打开系统已经配置好的 CAN 接口，按电机响应 ID 设置过滤器，并启动唯一的 `receiver_loop()` 线程。这里不发送运动命令。
4. `on_activate()` 逐个发送 A3 请求。线程收到应答后以多圈 Count 调用 `count_to_joint()`，全部有效才把命令缓存同步为反馈位置，避免控制器激活瞬间跳动。
5. `receiver_loop()` 只做收帧、协议解析和缓存更新。A2 更新速度，A3 更新位置和时间戳，AE 更新故障，B0 记录诊断参数；它不调用控制器，也不执行等待式请求。
6. `read()` 根据 `feedback_rate_hz` 节流发送 A3/A2 请求，然后检查位置缓存是否超时或故障。它不等待某个应答；反馈来自接收线程。
7. `write()` 校验每个命令是有限数，调用 `joint_to_count()` 检查减速比、方向和 int32 溢出，再发送 C2。`command_rate_limit_hz=0` 时不额外降低控制器更新频率。
8. `on_deactivate()` 只有 YAML 明确设置 `disable_output` 才发送 CF；默认不改变电机输出。`on_cleanup()` 停止线程、关闭 socket、清空缓存，析构函数也执行同样的线程安全收尾。

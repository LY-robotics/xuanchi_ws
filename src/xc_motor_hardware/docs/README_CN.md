# xc_motor_hardware

这是玄尺机械臂的通用 ROS 2 `ros2_control` SystemInterface 插件。插件名为 `xc_motor_hardware/SanpoCanSystem`，通过 Linux SocketCAN 使用 SANPO 自定义 CAN V3.07b0 协议控制任意数量关节。右臂和左臂共用同一套 C++ 驱动，只替换 YAML。

当前工作空间已有的 `xc_arm_description`、`xc_arm_moveit_config` 未被修改；原有 xacro 仍使用 `mock_components/GenericSystem`，真实硬件接入时再把硬件段替换为本包的插件和配置路径。

支持：经典 CAN、标准 11 bit ID、A2/A3/A4/AE/B0/C2/CF、position 命令、position/velocity 状态、多圈位置反馈、方向/减速比/零位配置、非阻塞接收线程和故障码缓存。

Windows 仅用于阅读、静态检查和编译不依赖 ROS 的协议/换算核心；ROS 2、pluginlib、SocketCAN、vcan0 必须在 Ubuntu 目标机验证。

本空间没有设置 `ROS_DISTRO`，所以文档不虚构已验证的发行版。源码使用 ROS 2 `hardware_interface::SystemInterface` 的通用生命周期接口；在目标机应先 `source /opt/ros/<distro>/setup.bash`，再按该发行版实际头文件完成构建。

## 快速使用

```bash
colcon build --packages-select xc_motor_hardware
source install/setup.bash
```

将 `examples/ros2_control_right_arm.xml` 中的绝对路径改为安装后的 YAML，并在 URDF 的 `<ros2_control>` 硬件段使用插件。控制器仍由现有 `joint_trajectory_controller` 和 `joint_state_broadcaster` 管理。

实机第一次验证请按 `SINGLE_ARM_QUICK_TEST_CN.md`，右臂控制器和 MoveIt 最小映射分别见 `examples/ros2_controllers_right_arm.yaml`、`examples/moveit_controllers_right_arm.yaml`。硬件测试不需要启动 MoveIt；启动 MoveIt 时，只映射已经启动的控制器。

## 安全边界

驱动不会自动发送零位命令，也不会默认在 `on_deactivate()` 发送 CF 自由停机。首次上电必须单关节、小角度、低风险验证，并准备急停。

> 已使用《自定义CAN通信协议_V3.07b0.pdf》核对 A2、A3、A4、AE、B0、C2、CF。Sanpo SPINE V8 的 SocketCAN 透传模式直接转发标准 CAN 帧，不使用 USB 协议模式的 ST/ET 封装；Linux 仍需先配置实际 `canX` 接口和 1 Mbps 波特率。模拟测试不能替代真实电机验证。

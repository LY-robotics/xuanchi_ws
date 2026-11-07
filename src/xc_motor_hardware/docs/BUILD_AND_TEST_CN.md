# 构建与测试

## Windows

当前环境检查结果：`ROS_DISTRO` 未设置，`ros2` 和 `g++` 不在 PATH，因此不能在本机声称 ROS 插件构建成功，也不能运行 SocketCAN/vcan。可审阅源码、YAML、XML 和文档；在安装 MinGW/clang 后可单独编译 `motor_protocol.cpp`、`joint_conversion.cpp` 与 `test_protocol_conversion.cpp`。

## Ubuntu + ROS 2

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
colcon build --packages-select xc_motor_hardware --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select xc_motor_hardware
colcon test-result --verbose
```

把 YAML 的 `can_interface` 改为 `vcan0` 后运行模拟节点，再启动 ros2_control。插件可通过 `pluginlib` 加载；最小验证是检查控制器能看到每个 joint 的 position 命令、position/velocity 状态接口。

右臂实机的分步流程见 `SINGLE_ARM_QUICK_TEST_CN.md`。右臂最小控制器参数见 `examples/ros2_controllers_right_arm.yaml`；若通过 MoveIt 发送轨迹，使用 `examples/moveit_controllers_right_arm.yaml`，并确保 controller_manager 中实际启动的名称也是 `right_arm_controller`。测试其它部位时，把 ros2_control XML、YAML、控制器 joints 和 MoveIt controller joints 配成同一个目标集合；不启动的控制器应从当前 controller_manager 参数中移除或注释。

本包的第一层测试覆盖 C2、小端 int32、A2/A3、故障码和非法 DLC；换算测试覆盖 1、3、4、4.2 减速比、反向、零位和往返误差。没有真实电机时，任何通过结果都只是协议/虚拟总线测试。

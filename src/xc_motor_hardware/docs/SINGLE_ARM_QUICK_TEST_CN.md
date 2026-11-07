# 右臂单臂快速验证

这个流程只验证右臂硬件、CAN 通信和 position 控制，不启动 MoveIt。第一次必须只接一台电机，确认接口和地址后再接五台。

## 1. 连接和识别 CAN 接口

目标机必须是 Ubuntu/Linux。SANPO SPINE V8 的 USB 转 CAN 透传在 Linux 中显示为 SocketCAN 的 `canX`；Windows 没有 SocketCAN 网络接口。接口编号不保证对应板上丝印，因此首次要逐个接口确认。参考：[SANPO SPINE V8 SocketCAN 文档](https://docs.sanporobot.com/v8/zh/socketcan.html)。

```bash
sudo apt update
sudo apt install can-utils
sudo modprobe gs_usb
ip link show type can
```

接好 USB、CANH、CANL、GND、终端电阻和电机电源后，把实际接口配置为经典 CAN 1 Mbps。下面假设接口是 `can0`，如果实际是 `can1`，所有命令和 YAML 都替换成 `can1`：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -s -details link show can0
```

不要在驱动的 `read()` 或 `write()` 中执行这些 `sudo ip link` 命令；接口由系统预先配置。

## 2. 先用 can-utils 读一个电机

右臂 Joint1 的设备地址是 34，主机请求 ID 是 `0x100 | 34 = 0x122`，应答 ID 是 `0x022`。先开监听：

```bash
candump can0
```

另一个终端发送无运动的查询：

```bash
cansend can0 122#A3
cansend can0 122#A2
cansend can0 122#AE
cansend can0 122#B0
```

也可以运行 `examples/right_arm_can_probe.sh can0 34` 自动发送这四个只读查询；脚本不会发送 C2、B1 或 CF。

预期：

- A3 返回 7 字节，`022#A3` 开头；Data[1..2] 是单圈 Count，Data[3..6] 是多圈有符号 Count。
- A2 返回 5 字节，速度单位为 0.01 RPM。
- AE 返回 8 字节，故障码为 Data[7]；正常静止时应为 0。
- B0 返回 7 字节，Data[2..5] 是小端单精度力矩常数，Data[6] 是协议减速比。

如果没有回复，先检查物理接口映射、设备地址、1 Mbps、终端电阻、CANH/CANL 和电机供电，不要立即发送 C2。

## 3. 发送最小风险的 C2 保持命令

记录 A3 多圈 Count 后，第一次 C2 应发送当前 Count，验证命令路径不会改变位置。例：当前 Count 为 `0x00001234`：

```bash
cansend can0 122#C234120000
```

协议规定 C2 应答的命令字仍为 C2，但后 6 个字节的位置字段与 A3 相同；驱动已同时接受 A3 和 C2 位置反馈。确认反馈仍正常后，才在机械安全、急停有效、单关节小行程条件下增加很小的 Count。不要用 B1 设置电机原点代替软件 YAML 的 `encoder_zero_count`，因为 B1 会把原点写入驱动板。

## 4. 加载右臂插件

右臂配置已包含五个关节和地址 34、57、14、32、21。复制配置到目标工作区后，把 `can_interface` 改为实际接口，并确认 `gear_ratio`、`direction`、`joint_offset_rad`、`encoder_zero_count` 已按现场标定填写。

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select xc_motor_hardware
source install/setup.bash
```

`examples/ros2_control_right_arm.xml` 需要被放入实际使用的 URDF/xacro `<ros2_control>` 段，硬件插件为 `xc_motor_hardware/SanpoCanSystem`，并通过 `config_file` 指向右臂 YAML。插件激活时会请求每个电机 A3，全部收到有效位置后才激活，并把 position 命令初值同步为反馈位置。

## 5. 单臂控制器和 MoveIt 的关系

单臂最小组合是三个集合完全一致的地方：

1. `<ros2_control>` XML 的 joint 名称和 position/position+velocity 接口；
2. YAML 的 `hardware.joints` 名称、ID 和机械参数；
3. `JointTrajectoryController` 的 `joints` 列表。

只想测试右臂时，controller_manager 只启动右臂 JTC 和 joint_state_broadcaster。其它臂、头部或手部控制器不要启动；如果它们写在同一个 controller YAML 中，可以注释/删除对应条目，或者使用本包提供的右臂专用控制器示例。控制器不能重复占用同一个 joint 的 position 命令接口。

MoveIt 不是纯硬件验证的必需项。通过 MoveIt 测试时，`moveit_controllers.yaml` 只保留当前实际启动的 FollowJointTrajectory 控制器名称和关节列表；规划组可以只选择右臂。未使用的 SRDF 规划组可以保留，它们不会被当前规划请求使用，但不要在 MoveIt 控制器映射里指向未启动的控制器。

因此，测试右臂时不需要修改整个 MoveIt 工程，只需使用右臂的 ros2_control XML、右臂 controller 参数和（若启动 MoveIt）右臂 MoveIt controller 映射。测试左臂或未来十/十二关节时，按同样规则替换三组关节集合和 YAML，C++ 插件不变。

## 6. 停止条件

出现无反馈、故障码非零、CAN 错误计数增加、Bus-Off、位置方向与预期相反或机械异常时，立即停止发送并使用急停。`deactivate_action: none` 是默认值，不能把软件生命周期切换当作机械制动。

# 真实硬件调试步骤

1. 检查 USB-CAN 驱动和 Linux 设备识别。
2. 确认实际 SocketCAN 接口名，预配置 1 Mbps。
3. 用 `ip -details link show can0` 和 `candump can0` 检查物理链路。
4. 只接一个电机，发送 A3 读取位置；记录多圈 Count。
5. 发送 B0，记录极对数、力矩常数和协议减速比，不能覆盖 YAML 的 4.2 等机械参数。
6. 验证输出轴一圈与电机侧 Count 的关系，确认 `motor_shaft` 或 `joint_output`。
7. 小范围验证方向，必要时把 YAML direction 改为 -1。
8. 设置 encoder_zero_count 和 joint_offset_rad，确认静止反馈。
9. 只发送一个小幅 C2，确认命令和反馈方向一致。
10. 逐个增加到五轴，再扩展另一只手臂。

全程准备急停，机械臂悬空或使用安全支撑；不能把 `on_deactivate()` 的 CF 当作安全制动。

## 右臂最快验证路径

完整的单臂步骤、具体 `cansend` 帧、控制器和 MoveIt 的最小配置见 `SINGLE_ARM_QUICK_TEST_CN.md`。推荐顺序是：只接一个电机 → A3/A2/AE/B0 只读 → 当前 Count 的 C2 保持命令 → 极小 Count 变化 → 再接其余四个右臂电机 → 最后启动 ros2_control 控制器。

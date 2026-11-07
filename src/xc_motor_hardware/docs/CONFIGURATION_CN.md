# 配置说明

YAML 顶层为 `hardware`。`can_interface` 默认 `can0`，`bitrate` 默认 1000000，支持 1000000、500000、250000、125000、100000（驱动不执行 ip link，接口须由系统预先配置），`encoder_counts_per_rev` 默认 16384，`position_count_reference` 可为 `motor_shaft` 或 `joint_output`，默认前者。

`feedback_rate_hz` 默认 20 Hz；`state_timeout_ms` 默认 250 ms；`response_timeout_ms` 默认 100 ms；`command_rate_limit_hz` 为 0 表示不额外限速；`deactivate_action` 默认 `none`，只有明确确认机械安全后才使用 `disable_output`。

每个 joint 必须有 `name`、`motor_id`、`gear_ratio`、`direction`。`motor_id` 为 1..254 且同一总线不能重复；`gear_ratio` 是电机侧转数/输出侧转数，可写 `4.2`；`direction` 只能为 1 或 -1；`joint_offset_rad` 单位 rad；`encoder_zero_count` 为软件 Count 零点。单关节还可覆盖 `position_count_reference`。

单关节还可设置 `min_position_rad` 和 `max_position_rad`（单位 rad，默认不限制）。write() 会在发送 C2 前拒绝有限范围之外的目标值。

当 `motor_shaft` 时：

`target_count = c0 + round(d * (q - q0) * r * N / (2π))`

`joint_position = d * (actual_count - c0) * 2π / (N * r) + q0`

当 `joint_output` 时，换算中的 `r` 按 1 处理。B0 返回的协议减速比只记录/诊断，不与 YAML 相乘。

右臂和左臂分别见 `right_arm.yaml`、`left_arm.yaml`。增加关节时同时增加 ros2_control XML 的 joint、YAML 项和控制器的 joints 列表；名称必须完全一致。双臂可以在同一个硬件实例中列出十个关节，也可以按物理总线拆成两个实例，但同一关节只能被一个控制器占用。

`upper_body_template.yaml` 只列出十个已知臂关节，头部条目被注释，因为题目没有提供头部 ID；取得真实 ID 和协议后再取消注释。

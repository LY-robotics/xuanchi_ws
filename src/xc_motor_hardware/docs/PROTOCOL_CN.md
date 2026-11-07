# 协议实现说明

## 基本帧

经典 CAN，标准帧，11 bit ID，DLC 最大 8，小端序。设备地址 1～254；0x00 广播、0xFF 公共地址不能作为普通 motor_id。主机请求 ID 为 `0x100 | motor_id`，应答 ID 为 `motor_id`。

## 已实现命令

|命令|请求 DLC|响应 DLC|数据|
|---|---:|---:|---|
|A2|1|5|Data0=A2，Data1..4 有符号 int32，0.01 RPM|
|A3|1|7|Data0=A3，Data1..2 uint16 单圈，Data3..6 int32 多圈，16384 Count/圈|
|A4|1|8|Data[1] 温度 1u ℃；Data[2..3] Q 轴电流 2s、0.001A；Data[4..5] 速度 2s、0.01 RPM；Data[6..7] 单圈角度 2u、16384 Count/圈|
|AE|1|8|Data[1..2] 母线电压 2u、0.01V；Data[3..4] 母线电流 2u、0.01A；Data[5] 温度 1u ℃；Data[6] 运行模式；Data[7] 故障码|
|B0|1|7|Data[1] 极对数 1u；Data[2..5] 力矩常数 4f、N/A；Data[6] 协议减速比 1u；不覆盖 YAML 浮点减速比|
|C2|5|7|请求 Data[0]=C2、Data[1..4] 为有符号 int32 Count；应答 Data[0]=C2，Data[1..6] 的位置字段与 A3 相同|
|CF|1|按配置发送|关闭输出/自由状态|

A2 的速度先除以 100 得 RPM，再由换算层按方向和减速比得到 rad/s。A3 优先使用多圈 Count，避免单圈回零跳变。AE 故障位识别 Bit0 电压、Bit1 电流、Bit2 温度、Bit3 编码器、Bit6 硬件、Bit7 软件；故障会使 `read()` 返回错误。

已根据《自定义CAN通信协议_V3.07b0.pdf》核对 A4、AE、B0：B0 是 DLC=7，力矩常数是小端 IEEE-754 单精度浮点数；温度字段是无符号字节。SocketCAN 透传模式不添加 USB 协议模式的 ST/ET 报头，Linux RAW CAN socket 直接收发上述标准帧。真实电机接入前仍需按调试流程验证设备地址、物理 CAN 接口和实测反馈。

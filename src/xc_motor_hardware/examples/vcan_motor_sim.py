#!/usr/bin/env python3
"""在 vcan0 上模拟一个或多个 SANPO 电机。

这只是协议/SocketCAN 集成测试节点，不代表真实电机行为。
"""
import argparse
import can

DEFAULT_IDS = (34, 57, 14, 32, 21)


def a3_frame(motor_id: int, count: int, command: int = 0xA3) -> can.Message:
    count = max(-(1 << 31), min((1 << 31) - 1, count))
    single = count % 16384
    payload = bytes([command]) + single.to_bytes(2, "little") + count.to_bytes(4, "little", signed=True)
    return can.Message(arbitration_id=motor_id, data=payload, is_extended_id=False)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="vcan0")
    parser.add_argument("--motor-ids", default="34,57,14,32,21",
                        help="逗号分隔的设备地址，例如 34,57,14,32,21")
    args = parser.parse_args()
    motor_ids = [int(item) for item in args.motor_ids.split(",") if item.strip()]
    if any(item < 1 or item > 254 for item in motor_ids):
        raise SystemExit("motor_id 必须在 1..254")
    positions = {motor_id: 0 for motor_id in motor_ids}
    bus = can.Bus(interface="socketcan", channel=args.channel)
    print(f"模拟电机已启动: {args.channel}, motor_ids={motor_ids}")
    try:
        for message in bus:
            motor_id = message.arbitration_id - 0x100
            if motor_id not in positions or not message.data:
                continue
            command = message.data[0]
            if command == 0xA2:
                response = can.Message(
                    arbitration_id=motor_id, data=bytes([0xA2, 0, 0, 0, 0]), is_extended_id=False)
            elif command == 0xA3:
                response = a3_frame(motor_id, positions[motor_id])
            elif command == 0xB0:
                # 极对数 7、力矩常数 1.0 N/A、协议减速比 3，仅用于软件回路测试。
                response = can.Message(
                    arbitration_id=motor_id, data=bytes([0xB0, 7, 0x00, 0x00, 0x80, 0x3F, 3]),
                    is_extended_id=False)
            elif command == 0xC2 and len(message.data) == 5:
                positions[motor_id] = int.from_bytes(message.data[1:5], "little", signed=True)
                # 模拟位置立即到达，便于验证 read() 的反馈闭环。
                response = a3_frame(motor_id, positions[motor_id], command=0xC2)
            else:
                continue
            bus.send(response)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()


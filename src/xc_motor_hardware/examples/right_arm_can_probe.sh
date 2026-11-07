#!/usr/bin/env bash
set -euo pipefail

# 只读查询右臂 Joint1（Dev_addr=34）的 A3/A2/AE/B0。
# 本脚本不会发送 C2、B1、CF，也不会设置 ip link。
CAN_IF="${1:-can0}"
MOTOR_ID="${2:-34}"
REQ_ID=$(printf '%03X' $((0x100 + MOTOR_ID)))

echo "监听 ${CAN_IF}，请求 ID=${REQ_ID}，设备地址=${MOTOR_ID}"
echo "另开终端运行：candump ${CAN_IF}"
cansend "${CAN_IF}" "${REQ_ID}#A3"
cansend "${CAN_IF}" "${REQ_ID}#A2"
cansend "${CAN_IF}" "${REQ_ID}#AE"
cansend "${CAN_IF}" "${REQ_ID}#B0"

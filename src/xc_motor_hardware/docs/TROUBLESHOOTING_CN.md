# vcan 模拟电机

`vcan_motor_sim.py` 需要 Ubuntu 的 python-can：

```bash
python3 -m pip install python-can
python3 examples/vcan_motor_sim.py --channel vcan0 --motor-id 34
```

它接收 A2/A3/B0/C2，并维护一个简单 Count；这是虚拟节点，不代表真实 SANPO 电机行为。A4、AE、B0 字段已按正式 PDF 修正，但仍需真实电机逐字段验证。

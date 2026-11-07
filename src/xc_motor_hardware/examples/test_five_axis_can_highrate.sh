#!/usr/bin/env bash
# SANPO V3.07b0 五轴 CAN 查询性能测试：只读 A3(位置)/A2(速度)，最高 500 Hz
# 用法：bash test_five_axis_can_highrate.sh [can0] [right|left] [all|200|250|300|350|400|450|500|250,300,350] [秒数] [both|position]
# 示例：bash test_five_axis_can_highrate.sh can0 right all 5 both
# Python 3 标准库，无需 ROS / python-can；从不发送电机使能/运动/失能指令。
set -euo pipefail
exec python3 - "$@" <<'PY'
import errno
import math
import select
import socket
import statistics
import struct
import sys
import time
from collections import Counter

MOTORS = {'right': [57, 34, 14, 32, 21], 'left': [58, 59, 31, 17, 19]}
RATES = [200, 250, 300, 350, 400, 450, 500]
MAX_HZ = 500
CMD_NAME = {0xA3: 'A3位置', 0xA2: 'A2速度'}
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF
SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
CAN_RAW_FILTER = getattr(socket, 'CAN_RAW_FILTER', 1)
CAN_RAW_RECV_OWN_MSGS = getattr(socket, 'CAN_RAW_RECV_OWN_MSGS', 4)
FRAME_STRUCT = struct.Struct('=IB3x8s')

# 预检要验证“每台电机、每种命令能独立应答”，不能把 10 条指令瞬间突发
# 预检和正式压力测试分开；放宽预检不能算作高频测试通过。
PRECHECK_RETRIES = 4
PRECHECK_TIMEOUT = 0.30
PRECHECK_GAP = 0.006


def now():
    return time.perf_counter()


def usage():
    print('用法: bash test_five_axis_can_highrate.sh [can0] [right|left] [all|250|300|350|400|450|500|250,300,350] [秒数] [both|position]')
    print('说明: all 依次测试 200、250、300、350、400、450、500 Hz；首次未达标停止后续升频')
    print('例子: bash test_five_axis_can_highrate.sh can0 right all 5 both')


def read_args(argv):
    if '-h' in argv or '--help' in argv:
        usage()
        sys.exit(0)
    if len(argv) > 5:
        raise ValueError('参数过多')
    iface = argv[0] if len(argv) > 0 else 'can0'
    side = argv[1] if len(argv) > 1 else 'right'
    step = argv[2] if len(argv) > 2 else 'all'
    try:
        seconds = float(argv[3]) if len(argv) > 3 else 3.0
    except ValueError:
        raise ValueError('测试秒数必须是数字') from None
    mode = argv[4] if len(argv) > 4 else 'both'
    if side not in MOTORS:
        raise ValueError('机械臂参数必须为 right 或 left')
    if step == 'all':
        rates = RATES
    else:
        try:
            rates = [int(item.strip()) for item in step.split(',')]
        except ValueError:
            raise ValueError('频率必须为 all、单个整数，或逗号分隔的整数，如 250,300,350') from None
        if not rates or len(rates) > 12 or len(set(rates)) != len(rates) or any(hz < 10 or hz > MAX_HZ for hz in rates):
            raise ValueError(f'频率范围必须是 10～{MAX_HZ} Hz，且不能重复（最多 12 档）')
        if rates != sorted(rates):
            raise ValueError('测试频率应从低到高排列')
    if not math.isfinite(seconds) or not (0.5 <= seconds <= 120):
        raise ValueError('每档测试秒数应在 0.5～120 之间')
    if mode not in ('both', 'position'):
        raise ValueError('模式必须为 both 或 position')
    commands = (0xA3, 0xA2) if mode == 'both' else (0xA3,)
    return iface, side, rates, seconds, commands


def make_socket(iface, ids):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        # 标准帧 ID=电机地址；扩展帧、本机 0x1xx 请求不能匹配过滤器。
        filters = b''.join(struct.pack('=II', mid, CAN_SFF_MASK | CAN_EFF_FLAG) for mid in ids)
        s.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filters)
        s.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, struct.pack('=I', 0))
        s.setblocking(False)
        s.bind((iface,))
        return s
    except Exception:
        s.close()
        raise


def send_one(sock, motor_id, cmd):
    # 请求：CAN ID=0x100|Dev_addr，DLC=1，字节 A3 或 A2。
    frame = FRAME_STRUCT.pack(0x100 | motor_id, 1, bytes((cmd,)) + bytes(7))
    n = sock.send(frame)
    if n != FRAME_STRUCT.size:
        raise OSError(errno.EIO, '发送 CAN 帧长度不足')


def recv_one(sock):
    raw = sock.recv(FRAME_STRUCT.size)
    if len(raw) != FRAME_STRUCT.size:
        return None
    can_id, dlc, data = FRAME_STRUCT.unpack(raw)
    if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None
    mid = can_id & CAN_SFF_MASK
    if not (1 <= mid <= 254) or not (1 <= dlc <= 8):
        return None
    cmd = data[0]
    if (cmd == 0xA3 and dlc == 7) or (cmd == 0xA2 and dlc == 5):
        return (mid, cmd)
    return None


def drain(sock, max_seconds):
    end = now() + max_seconds
    count = 0
    while now() < end:
        readable, _, _ = select.select([sock], [], [], max(0.0, min(0.002, end - now())))
        if not readable:
            continue
        while True:
            try:
                sock.recv(FRAME_STRUCT.size)
                count += 1
            except BlockingIOError:
                break
    return count


def collect(sock, expected, wait_seconds):
    pending = set(expected)
    seen = set()
    unexpected = 0
    duplicates = 0
    last_read = None
    end = now() + wait_seconds
    while pending:
        wait = end - now()
        if wait <= 0:
            break
        readable, _, _ = select.select([sock], [], [], wait)
        if not readable:
            break
        while True:
            try:
                key = recv_one(sock)
            except BlockingIOError:
                break
            stamp = now()
            if key in pending:
                pending.remove(key)
                seen.add(key)
                last_read = stamp
            elif key in seen:
                duplicates += 1
            else:
                unexpected += 1
            if not pending:
                break
    return pending, last_read, unexpected, duplicates


def preflight(sock, ids, commands):
    expected_count = len(ids) * len(commands)
    print(f'\n[预检] 分别查询 {expected_count} 个“电机 × 指令”组合；每项最多 {PRECHECK_RETRIES} 次，独立等待 {PRECHECK_TIMEOUT*1000:.0f}ms。', flush=True)
    print('       与原脚本不同：不会一次突发发送 10 条查询。')
    missing = []
    delays = []
    for mid in ids:
        for cmd in commands:
            key = (mid, cmd)
            success = False
            for attempt in range(1, PRECHECK_RETRIES + 1):
                # 清除上一条查询的迟到帧，不把它误算作当前查询的应答。
                drain(sock, 0.006)
                start = now()
                send_one(sock, mid, cmd)
                pending, last, _, _ = collect(sock, (key,), PRECHECK_TIMEOUT)
                if not pending:
                    latency_ms = (last - start) * 1000
                    delays.append(latency_ms)
                    print(f'  [OK]   ID={mid:<3} {CMD_NAME[cmd]}   第{attempt}次应答  {latency_ms:7.3f} ms', flush=True)
                    success = True
                    break
                if attempt < PRECHECK_RETRIES:
                    drain(sock, 0.02)
            if not success:
                missing.append(key)
                print(f'  [FAIL] ID={mid:<3} {CMD_NAME[cmd]}   {PRECHECK_RETRIES} 次均未收到合法应答', flush=True)
            time.sleep(PRECHECK_GAP)
    if missing:
        print(f'\n[预检失败] 成功 {expected_count-len(missing)}/{expected_count}；仍缺少：')
        for mid, cmd in missing:
            print(f'    ID={mid} {CMD_NAME[cmd]}')
        print('这些项目单独重试仍失败；请检查同一接口的实际回复、USB掉线和其他进程占用。')
        print('为了避免产生误导性的性能结果，本次停止；不会把缺失应答当作正常轮次。')
        return False
    print(f'[预检通过] 全部 {expected_count}/{expected_count} 项成功；单次应答平均 {statistics.mean(delays):.3f} ms。', flush=True)
    drain(sock, 0.03)
    return True


def percentile(values, p):
    if not values:
        return float('nan')
    ordered = sorted(values)
    idx = (len(ordered) - 1) * p / 100
    low, high = math.floor(idx), math.ceil(idx)
    return ordered[low] + (ordered[high] - ordered[low]) * (idx - low)


def send_round_spaced(sock, ids, commands, period):
    # 平滑发送同一轮请求，减少 gs_usb / 电机邮箱在一个突发里的拥堵。
    # 高频时自动缩短间隔；绝不将 200Hz 伪装成低频测试。
    requests = [(mid, cmd) for mid in ids for cmd in commands]
    gap = min(0.004, period / (2.0 * len(requests)))
    start = now()
    for i, (mid, cmd) in enumerate(requests):
        due = start + i * gap
        pause = due - now()
        if pause > 0:
            time.sleep(pause)
        send_one(sock, mid, cmd)
    return start, gap


def test_rate(sock, ids, commands, hz, seconds):
    expected = [(mid, cmd) for mid in ids for cmd in commands]
    period = 1.0 / hz
    # 严格按本轮开始后的一个周期截止接收，避免把截止后的回复算成按时。
    # A2/A3 无请求序号；超时后强制排空以降低跨轮误认。
    drain(sock, 0.02)
    finish = now() + seconds
    next_due = now()
    started = 0
    completed = 0
    completed_on_time = 0
    complete_times = []
    missing_keys = Counter()
    unexpected_total = 0
    duplicate_total = 0
    first_start = None
    last_end = None
    while now() < finish:
        pause = next_due - now()
        if pause > 0:
            time.sleep(pause)
        if now() >= finish:
            break
        start, gap = send_round_spaced(sock, ids, commands, period)
        if first_start is None:
            first_start = start
        started += 1
        deadline = start + period
        missing, last, unexpected, duplicates = collect(sock, expected, max(0.0, deadline - now()))
        unexpected_total += unexpected
        duplicate_total += duplicates
        if not missing and last is not None:
            completed += 1
            duration_ms = (last - start) * 1000
            complete_times.append(duration_ms)
            if duration_ms <= 1000.0 / hz:
                completed_on_time += 1
        else:
            missing_keys.update(missing)
            # SANPO A2/A3 没有序列号，超时后等待静默以减小迟到包跨轮错配。
            drain(sock, 0.02)
        last_end = now()
        next_due = max(next_due + period, last_end)
    if not started:
        return None
    elapsed = max(last_end - first_start, finish - first_start, 1e-9)
    sent_hz = started / elapsed
    complete_hz = completed / elapsed
    complete_percent = 100.0 * completed / started
    timely_percent = 100.0 * completed_on_time / started
    avg = statistics.mean(complete_times) if complete_times else float('nan')
    p95 = percentile(complete_times, 95)
    slowest = max(complete_times) if complete_times else float('nan')
    print(f'{hz:>3} Hz | {started:>4}轮 | 完整 {completed:>4} ({complete_percent:>5.1f}%) | '
          f'按时 {completed_on_time:>4} ({timely_percent:>5.1f}%) | '
          f'实发 {sent_hz:>6.1f} Hz | 完整速率 {complete_hz:>6.1f} Hz | '
          f'时延均值 {avg:>7.2f} / P95 {p95:>7.2f} / 最慢 {slowest:>7.2f} ms', flush=True)
    if missing_keys:
        detail = '、'.join(f'ID{mid}-{cmd:02X}:{cnt}' for (mid, cmd), cnt in sorted(missing_keys.items()))
        print(f'      缺少应答：{detail}', flush=True)
    if unexpected_total or duplicate_total:
        print(f'      无法匹配/格式异常帧={unexpected_total}，重复应答={duplicate_total}', flush=True)
    passed = (complete_percent >= 99.0 and timely_percent >= 99.0 and complete_hz >= 0.95 * hz)
    return hz, passed, complete_percent, timely_percent, complete_hz, p95


def main():
    try:
        iface, side, rates, seconds, commands = read_args(sys.argv[1:])
    except ValueError as exc:
        print(f'参数错误：{exc}', file=sys.stderr)
        usage()
        return 2
    ids = MOTORS[side]
    print('=' * 66)
    print(f'SANPO 五轴 CAN 高频压力测试 · 只读（最高 {MAX_HZ} Hz）')
    print(f'接口={iface} | 机械臂={side} | ID={ids}')
    print(f'频率={rates} Hz，每档 {seconds:g}s，查询={"A3位置+A2速度" if len(commands)==2 else "A3位置"}')
    print('协议：CAN 标准帧，请求 ID=0x100|电机ID，应答 ID=电机ID；A3 DLC7，A2 DLC5')
    print('提示：停掉其他正在访问这些电机的轮询程序；本脚本绝不发送运动指令。')
    print('说明：完整反馈必须在对应周期内收齐；500Hz每轮仅 2ms。')
    print('      连续升频时首档未达标即停止；不要将 TX 发送速率误当作 RX 实际完成速率。')
    print('=' * 66, flush=True)
    try:
        with make_socket(iface, ids) as sock:
            if not preflight(sock, ids, commands):
                return 1
            print('\n目标频率 | 轮数 | 完整率 | 周期按时率 | 实发频率 | 实际完整频率 | 收齐时延', flush=True)
            results = []
            for hz in rates:
                result = test_rate(sock, ids, commands, hz, seconds)
                if result:
                    results.append(result)
                    if not result[1] and len(rates) > 1:
                        print(f'[停止升频] {hz} Hz 未达到完整率/按时率/实际完成频率门槛；后续档位跳过。', flush=True)
                        break
                drain(sock, 0.10)
            print('\n===== 结果汇总 =====')
            stable = [r[0] for r in results if r[1]]
            if stable:
                print(f'暂定标准（完整率≥99%，按时率≥99%，实际完整频率≥目标95%），最高通过：{max(stable)} Hz')
            else:
                print('没有频率档位达到暂定标准；请检查缺失应答和实际完成频率。')
            print(f'建议额外检查：ip -details -statistics link show {iface}')
            print('注意：Linux 用户态 + USB-CAN 不保证硬实时；CAN 无查询序号，迟到应答仍可能影响统计。')
            print('      本测试不发送 C2 位置命令；结果不能等同 ros2_control 运动控制频率。')
            return 0
    except OSError as exc:
        print(f'\n[CAN 错误] {exc}', file=sys.stderr)
        print(f'请检查 {iface} 是否仍为 UP，SANPO V4 USB 是否断开，以及 dmesg 日志。', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n用户已停止测试。')
        return 130


if __name__ == '__main__':
    sys.exit(main())
PY

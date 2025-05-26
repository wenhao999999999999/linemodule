import time
import threading
from pymodbus.client.sync import ModbusSerialClient

# === 编码器零点绝对位置（实测） ===
ZERO_POSITIONS = {
    'X1': -7658001,
    'X2': -545689,
    'Z':  5839174,
    'Y':  -2140820
}

# === 每个轴的串口设备路径 ===
PORTS = {
    'X1': '/dev/line_module_bx1',
    'X2': '/dev/line_module_bx2',
    'Z':  '/dev/line_module_bz',
    'Y':  '/dev/line_module_by'
}

RESOLUTION = 131072
SCREW_PITCH = 10.0  # mm
TOLERANCE_MM = 1.0
TIMEOUT_SEC = 60
SPEED_RPM = -600

def read_i32(client, addr):
    rr = client.read_holding_registers(addr, 2, unit=1)
    if rr.isError() or not hasattr(rr, 'registers'):
        return None
    low, high = rr.registers[0], rr.registers[1]
    value = (high << 16) | low
    if value >= 0x80000000:
        value -= 0x100000000
    return value

def write_u16(client, addr, val):
    client.write_register(addr, val, unit=1)
    time.sleep(0.02)

def write_checked(client, addr, val, name, axis):
    write_u16(client, addr, val)
    rr = client.read_holding_registers(addr, 1, unit=1)
    if rr.isError() or rr.registers[0] != (val & 0xFFFF):
        print(f"[{axis}] ❌ 写入失败: {name}")
        return False
    print(f"[{axis}] ✅ 写入成功: {name} = {val}")
    return True

def home_axis(axis, port, zero_pos):
    client = ModbusSerialClient(method='rtu', port=port, baudrate=57600, parity='N',
                                stopbits=1, bytesize=8, timeout=1)
    if not client.connect():
        print(f"[{axis}] ❌ 无法连接 {port}")
        return

    print(f"[{axis}] ⚙️ 初始化中...")

    # 初始化
    write_u16(client, 773, 1)
    time.sleep(0.1)
    write_u16(client, 773, 0)
    write_u16(client, 771, 0)

    ok = (
        write_checked(client, 1801, 100, "H07_09 正转矩限制", axis) and
        write_checked(client, 1802, 100, "H07_10 负转矩限制", axis) and
        write_checked(client, 512, 0, "H02_00 速度模式", axis) and
        write_checked(client, 770, 1, "H03_02 启用伺服", axis) and
        write_checked(client, 771, 0, "H03_03 正逻辑", axis) and
        write_checked(client, 1538, 0, "H06_02 内部速度指令", axis) and
        write_checked(client, 1539, SPEED_RPM & 0xFFFF, "H06_03 回零速度", axis) and
        write_checked(client, 1541, 200, "H06_05 加速时间", axis) and
        write_checked(client, 1542, 200, "H06_06 减速时间", axis)
    )
    if not ok:
        print(f"[{axis}] ❌ 参数写入失败")
        return

    write_u16(client, 771, 1)
    print(f"[{axis}] 🚦 开始回零")

    start_time = time.time()
    last_pos = None

    while True:
        if time.time() - start_time > TIMEOUT_SEC:
            print(f"[{axis}] ⏰ 回零超时，强制停止")
            write_u16(client, 1539, 0)
            break

        abs_pos = read_i32(client, 2893)
        if abs_pos is None:
            continue

        delta_pulse = abs(abs_pos - zero_pos)
        delta_mm = delta_pulse / RESOLUTION * SCREW_PITCH
        print(f"[{axis}] 📍 当前绝对位置: {abs_pos}, 偏差: {delta_mm:.4f} mm")

        if last_pos is not None:
            if last_pos < zero_pos and abs_pos >= zero_pos:
                print(f"[{axis}] ⛔ 检测到跳过目标，立即停止")
                write_u16(client, 1539, 0)
                break

        if delta_mm < TOLERANCE_MM:
            print(f"[{axis}] ✅ 回零完成，进入容差范围")
            write_u16(client, 1539, 0)
            break

        last_pos = abs_pos
        time.sleep(0.05)

    # 恢复工作力矩
    write_checked(client, 1801, 200, "H07_09 恢复正转矩", axis)
    write_checked(client, 1802, 200, "H07_10 恢复负转矩", axis)

    client.close()
    print(f"[{axis}] ✅ 回零完成")

def main():
    threads = []
    for axis in ['X1', 'X2', 'Z', 'Y']:
        t = threading.Thread(target=home_axis,
                             args=(axis, PORTS[axis], ZERO_POSITIONS[axis]),
                             daemon=True)
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    print("✅ 所有轴回零完成")

if __name__ == '__main__':
    main()

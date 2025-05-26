import time
from pymodbus.client.sync import ModbusSerialClient

def read_i32(client, addr):
    rr = client.read_holding_registers(addr, 2, unit=1)
    if rr.isError() or not hasattr(rr, 'registers'):
        return None
    low, high = rr.registers[0], rr.registers[1]  # 小端模式
    value = (high << 16) | low
    if value >= 0x80000000:
        value -= 0x100000000
    return value

def write_u16(client, addr, val):
    client.write_register(addr, val, unit=1)
    time.sleep(0.02)

def write_checked(client, addr, val, name):
    write_u16(client, addr, val)
    rr = client.read_holding_registers(addr, 1, unit=1)
    if rr.isError() or rr.registers[0] != (val & 0xFFFF):
        print(f"❌ 写入失败: {name}")
        return False
    print(f"✅ 写入成功: {name} = {val}")
    return True

def home_line_module_b(port, zero_pos, screw_pitch=10.0, resolution=131072):
    client = ModbusSerialClient(method='rtu', port=port, baudrate=57600, parity='N',
                                stopbits=1, bytesize=8, timeout=1)
    if not client.connect():
        print(f"❌ 无法连接 {port}")
        return

    # 初始化参数
    write_u16(client, 773, 1)  # 清除故障
    time.sleep(0.1)
    write_u16(client, 773, 0)
    write_u16(client, 771, 0)

    ok = (
        write_checked(client, 1801, 100, "回零正转矩限制 H07_09") and
        write_checked(client, 1802, 100, "回零负转矩限制 H07_10") and
        write_checked(client, 512, 0, "速度模式 H02_00") and
        write_checked(client, 770, 1, "DI1 启用伺服") and
        write_checked(client, 771, 0, "DI1 正逻辑") and
        write_checked(client, 1538, 0, "内部速度控制") and
        write_checked(client, 1539, (-600) & 0xFFFF, "启动回零速度 H06_03") and
        write_checked(client, 1541, 200, "加速时间 H06_05") and
        write_checked(client, 1542, 200, "减速时间 H06_06")
    )
    if not ok:
        print("❌ 参数写入失败")
        return

    write_u16(client, 771, 1)  # 启动伺服
    time.sleep(1)

    print("🚦 开始回零")

    start_time = time.time()
    timeout = 60  # 秒
    tolerance_mm = 1.0
    last_pos = None

    while True:
        if time.time() - start_time > timeout:
            print("⏰ 回零超时，强制停止电机")
            write_u16(client, 1539, 0)
            break

        abs_pos = read_i32(client, 2893)
        if abs_pos is None:
            continue

        delta_pulse = abs(abs_pos - zero_pos)
        delta_mm = delta_pulse / resolution * screw_pitch
        print(f"当前位置: {abs_pos}, 偏差: {delta_mm:.4f} mm")

        # 判定是否跳过目标
        if last_pos is not None:
            if last_pos < zero_pos and abs_pos >= zero_pos:
                print("⛔ 检测到跳过目标位置，立即停止")
                write_u16(client, 1539, 0)
                break

        if delta_mm < tolerance_mm:
            print("✅ 偏差进入容差范围内，立即停止")
            write_u16(client, 1539, 0)
            break

        last_pos = abs_pos
        time.sleep(0.05)

    # 恢复工作转矩
    write_checked(client, 1801, 200, "恢复工作正转矩限制 H07_09")
    write_checked(client, 1802, 200, "恢复工作负转矩限制 H07_10")

    client.close()
    print("✅ 模组 B 回零流程结束")

if __name__ == "__main__":
    TARGET_ZERO_POSITION = -2150000  # 根据你的电机标定值设置
    PORT = '/dev/line_module_by'
    home_line_module_b(PORT, TARGET_ZERO_POSITION)

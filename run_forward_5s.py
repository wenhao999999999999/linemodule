from pymodbus.client.sync import ModbusSerialClient
import time
import threading

# === 参数配置 ===
PORTS = {
    # === B 模组 ===
    'BX1': '/dev/line_module_bx1',
    'BX2': '/dev/line_module_bx2',
    'BY':  '/dev/line_module_by',
    'BZ':  '/dev/line_module_bz',

    # === A 模组 ===
    'AX1': '/dev/line_module_ax1',
    'AX2': '/dev/line_module_ax2',
    'AY':  '/dev/line_module_ay',
    'AZ':  '/dev/line_module_az',
}
BAUDRATE = 57600
MODBUS_UNIT = 1
TARGET_SPEED = 600  # 正向转速 rpm
RUN_TIME_SEC = 2    # 运行时长

# === 寄存器地址 ===
REG_MODE = 512
REG_ENABLE = 771
REG_SPEED_SRC = 1538
REG_SPEED_VALUE = 1539
REG_ACCEL = 1541
REG_DECEL = 1542
REG_CLEAR_ALARM = 773

def write_u16_checked(client, addr, val, name, axis_name):
    client.write_register(addr, val, unit=MODBUS_UNIT)
    time.sleep(0.02)
    rr = client.read_holding_registers(addr, 1, unit=MODBUS_UNIT)
    if rr.isError() or rr.registers[0] != (val & 0xFFFF):
        print(f"[{axis_name}] ❌ 写入失败: {name}")
        return False
    print(f"[{axis_name}] ✅ 写入成功: {name} = {val}")
    return True

def control_axis(axis_name, port):
    client = ModbusSerialClient(method='rtu', port=port, baudrate=BAUDRATE,
                                parity='N', stopbits=1, bytesize=8, timeout=1)
    if not client.connect():
        print(f"[{axis_name}] ❌ 无法连接 {port}")
        return

    try:
        print(f"[{axis_name}] ⚙️ 初始化中...")
        write_u16_checked(client, REG_CLEAR_ALARM, 1, "清除报警", axis_name)
        time.sleep(0.1)
        write_u16_checked(client, REG_CLEAR_ALARM, 0, "清除报警结束", axis_name)

        ok = (
            write_u16_checked(client, REG_MODE, 0, "H02_00 = 0 → 速度模式", axis_name) and
            write_u16_checked(client, REG_SPEED_SRC, 0, "H06_02 = 0 → 内部速度源", axis_name) and
            write_u16_checked(client, REG_SPEED_VALUE, TARGET_SPEED, f"H06_03 = {TARGET_SPEED} rpm", axis_name) and
            write_u16_checked(client, REG_ACCEL, 300, "H06_05 = 30ms 加速", axis_name) and
            write_u16_checked(client, REG_DECEL, 300, "H06_06 = 30ms 减速", axis_name)
        )
        if not ok:
            print(f"[{axis_name}] ❌ 参数设置失败")
            return

        write_u16_checked(client, REG_ENABLE, 1, "H03_03 = 1 → 启动伺服", axis_name)
        print(f"[{axis_name}] 🚀 正向运动中...")
        time.sleep(RUN_TIME_SEC)

        write_u16_checked(client, REG_SPEED_VALUE, 0, "H06_03 = 0 → 停止", axis_name)
        print(f"[{axis_name}] 🛑 已停止")

    finally:
        client.close()
        print(f"[{axis_name}] 🔌 已断开连接")

def main():
    threads = []
    for axis_name, port in PORTS.items():
        t = threading.Thread(target=control_axis, args=(axis_name, port))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    print("✅ 所有轴执行完毕")

if __name__ == '__main__':
    main()

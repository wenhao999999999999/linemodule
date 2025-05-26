from pymodbus.client.sync import ModbusSerialClient
import time

# === 配置 ===
SERIAL_PORT = '/dev/line_module_by'  # 修改为你的轴串口名
BAUDRATE = 57600
MODBUS_UNIT = 1  # 站号
TARGET_SPEED = 600  # 正转转速（rpm）

# === 寄存器地址 ===
REG_MODE = 512         # H02_00：控制模式（0=速度）
REG_ENABLE = 771       # H03_03：伺服使能
REG_SPEED_SRC = 1538   # H06_02：速度源（0=内部）
REG_SPEED_VALUE = 1539 # H06_03：目标速度
REG_ACCEL = 1541       # H06_05：加速
REG_DECEL = 1542       # H06_06：减速
REG_CLEAR_ALARM = 773  # 清除报警

def write_u16_checked(client, addr, val, name):
    client.write_register(addr, val, unit=MODBUS_UNIT)
    time.sleep(0.02)
    rr = client.read_holding_registers(addr, 1, unit=MODBUS_UNIT)
    if rr.isError() or rr.registers[0] != val:
        print(f"❌ 写入失败: {name}")
        return False
    print(f"✅ 写入成功: {name} = {val}")
    return True

def main():
    client = ModbusSerialClient(method='rtu', port=SERIAL_PORT, baudrate=BAUDRATE,
                                parity='N', stopbits=1, bytesize=8, timeout=1)
    if not client.connect():
        print("❌ 无法连接驱动器")
        return

    try:
        # 清除报警
        write_u16_checked(client, REG_CLEAR_ALARM, 1, "清除报警")
        time.sleep(0.1)
        write_u16_checked(client, REG_CLEAR_ALARM, 0, "清除报警结束")

        # 设置速度模式参数
        ok = (
            write_u16_checked(client, REG_MODE, 0, "H02_00 = 0 → 速度模式") and
            write_u16_checked(client, REG_SPEED_SRC, 0, "H06_02 = 0 → 内部速度源") and
            write_u16_checked(client, REG_SPEED_VALUE, TARGET_SPEED, f"H06_03 = {TARGET_SPEED} rpm") and
            write_u16_checked(client, REG_ACCEL, 300, "H06_05 = 30ms 加速") and
            write_u16_checked(client, REG_DECEL, 300, "H06_06 = 30ms 减速")
        )
        if not ok:
            print("❌ 速度模式设置失败")
            return

        # 启用伺服
        write_u16_checked(client, REG_ENABLE, 1, "H03_03 = 1 → 启动伺服")

        print("🚀 正向运行 5 秒")
        time.sleep(3)

        # 停止运动
        write_u16_checked(client, REG_SPEED_VALUE, 0, "H06_03 = 0 → 停止")
        print("🛑 已停止运动")

    finally:
        client.close()

if __name__ == '__main__':
    main()

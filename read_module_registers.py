from pymodbus.client.sync import ModbusSerialClient

# A 模组 & B 模组所有轴及其串口
AXIS_PORTS = {
    # B 模组
    'bx1': '/dev/line_module_bx1',
    'bx2': '/dev/line_module_bx2',
    'by':  '/dev/line_module_by',
    'bz':  '/dev/line_module_bz',
    # A 模组
    'ax1': '/dev/line_module_ax1',
    'ax2': '/dev/line_module_ax2',
    'ay':  '/dev/line_module_ay',
    'az':  '/dev/line_module_az',
}

SLAVE_ID = 1  # 默认从站地址

def modbus_address(hname: str) -> int:
    hname = hname.upper().replace("H", "")
    group_hex, offset_dec = hname.split("_")
    return int(group_hex, 16) * 256 + int(offset_dec, 10)

# 寄存器地址
ADDR_H0B_70 = modbus_address("H0B_70")  # Int16
ADDR_H0B_71 = modbus_address("H0B_71")  # UInt32
ADDR_H0B_77 = modbus_address("H0B_77")  # Int32

def read_i16(client, addr):
    rr = client.read_holding_registers(addr, 1, unit=SLAVE_ID)
    if rr.isError():
        return None
    val = rr.registers[0]
    return val - 0x10000 if val >= 0x8000 else val

def read_u32(client, addr, low_first=True):
    rr = client.read_holding_registers(addr, 2, unit=SLAVE_ID)
    if rr.isError() or not hasattr(rr, 'registers'):
        return None
    low, high = (rr.registers[0], rr.registers[1]) if low_first else (rr.registers[1], rr.registers[0])
    return (high << 16) | low

def read_i32(client, addr, low_first=True):
    rr = client.read_holding_registers(addr, 2, unit=SLAVE_ID)
    if rr.isError() or not hasattr(rr, 'registers'):
        return None
    low, high = (rr.registers[0], rr.registers[1]) if low_first else (rr.registers[1], rr.registers[0])
    value = (high << 16) | low
    if value >= 0x80000000:
        value -= 0x100000000
    return value

def read_axis(axis_name, port):
    client = ModbusSerialClient(method='rtu', port=port, baudrate=57600,
                                parity='N', stopbits=1, bytesize=8, timeout=1)
    if not client.connect():
        print(f"[{axis_name.upper()}] ❌ 无法连接串口 {port}")
        return

    try:
        circle = read_i16(client, ADDR_H0B_70)
        single_turn = read_u32(client, ADDR_H0B_71)
        absolute = read_i32(client, ADDR_H0B_77)

        print(f"\n[{axis_name.upper()}] 📊 数据：")
        print(f"  🔁 圈数 H0B_70  = {circle}")
        print(f"  🔄 单圈 H0B_71  = {single_turn}")
        print(f"  🧭 绝对 H0B_77  = {absolute}")

    finally:
        client.close()

def main():
    print("📦 正在读取 A/B 模组所有轴的寄存器...\n")
    for axis, port in AXIS_PORTS.items():
        read_axis(axis, port)
    print("\n✅ 完成")

if __name__ == '__main__':
    main()

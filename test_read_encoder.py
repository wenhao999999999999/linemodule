from pymodbus.client.sync import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian

client = ModbusSerialClient(method='rtu', port='/dev/line_module_y', baudrate=57600, timeout=1)
client.connect()

try:
    print("🔍 正在读取地址 2893（H0B_77） ...")
    result = client.read_holding_registers(2893, 2, unit=1)
    if result.isError():
        print("❌ 读取失败：", result)
    else:
        print("✅ 原始寄存器值:", result.registers)
        decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Little, wordorder=Endian.Little)
        pos = decoder.decode_32bit_int()
        print(f"✅ 解码后当前位置 = {pos}")
finally:
    client.close()


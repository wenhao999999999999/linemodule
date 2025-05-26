from pymodbus.client.sync import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian
import time

client = ModbusSerialClient(method='rtu', port='/dev/line_module_y', baudrate=57600, timeout=1)
client.connect()

while True:
    result = client.read_holding_registers(2823, 2, unit=1)
    if result.isError():
        print(f"读取失败: {result}")
    else:
        print(f"原始寄存器值: {result.registers}")
        decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Little, wordorder=Endian.Little)
        pos = decoder.decode_32bit_int()
        print(f"当前反馈位置（H0B_07）= {pos}")
    time.sleep(0.5)


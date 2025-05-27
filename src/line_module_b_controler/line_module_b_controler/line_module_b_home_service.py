#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
import time
from pymodbus.client.sync import ModbusSerialClient

# === 零点位置（实测） ===
ZERO_POSITIONS = {
    'X1': -7696697,
    'X2': -597383,
    'Z':  5778817,
    'Y':  -2168266 
}

# === 对应串口设备 ===
PORTS = {
    'X1': '/dev/line_module_bx1',
    'X2': '/dev/line_module_bx2',
    'Z':  '/dev/line_module_bz',
    'Y':  '/dev/line_module_by'
}

RESOLUTION = 131072
SCREW_PITCH = 10.0
TOLERANCE_MM = 5.0
TIMEOUT_SEC = 60
SPEED_RPM = -600

class BModuleHomeService(Node):
    def __init__(self):
        super().__init__('b_module_home_service')
        self.srv = self.create_service(Trigger, '/line_module_b_home', self.callback)
        self.get_logger().info("🟢 B模组四轴回零服务已启动")
        self.results = {}

    def callback(self, request, response):
        threads = []
        self.results.clear()

        for axis in ['X1', 'X2', 'Z', 'Y']:
            t = threading.Thread(target=self.home_axis, args=(axis,))
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        msg = "\n".join(f"[{k}] {v}" for k, v in self.results.items())
        success = all("✅" in v for v in self.results.values())
        response.success = success
        response.message = f"四轴回零结果：\n{msg}"
        return response

    def read_i32(self, client, addr):
        rr = client.read_holding_registers(addr, 2, unit=1)
        if rr.isError() or not hasattr(rr, 'registers'):
            return None
        low, high = rr.registers[0], rr.registers[1]
        value = (high << 16) | low
        if value >= 0x80000000:
            value -= 0x100000000
        return value

    def read_i16(self, client, addr):
        rr = client.read_holding_registers(addr, 1, unit=1)
        if rr.isError():
            return None
        val = rr.registers[0]
        return val - 0x10000 if val >= 0x8000 else val

    def read_u32(self, client, addr):
        rr = client.read_holding_registers(addr, 2, unit=1)
        if rr.isError() or not hasattr(rr, 'registers'):
            return None
        low, high = rr.registers[0], rr.registers[1]
        return (high << 16) | low

    def write_u16(self, client, addr, val):
        client.write_register(addr, val, unit=1)
        time.sleep(0.02)

    def write_checked(self, client, addr, val, name, axis):
        self.write_u16(client, addr, val)
        rr = client.read_holding_registers(addr, 1, unit=1)
        if rr.isError() or rr.registers[0] != (val & 0xFFFF):
            self.get_logger().error(f"[{axis}] ❌ 写入失败: {name}")
            return False
        self.get_logger().info(f"[{axis}] ✅ 写入成功: {name} = {val}")
        return True

    def home_axis(self, axis):
        port = PORTS[axis]
        zero_pos = ZERO_POSITIONS[axis]
        result = "⚠️ 未执行"

        client = ModbusSerialClient(method='rtu', port=port, baudrate=57600, parity='N',
                                    stopbits=1, bytesize=8, timeout=1)
        if not client.connect():
            result = f"❌ 串口连接失败 {port}"
            self.results[axis] = result
            return

        self.get_logger().info(f"[{axis}] ⚙️ 开始初始化")
        try:
            self.write_u16(client, 773, 1)
            time.sleep(0.1)
            self.write_u16(client, 773, 0)
            self.write_u16(client, 771, 0)

            ok = (
                self.write_checked(client, 1801, 100, "H07_09 正转矩限制", axis) and
                self.write_checked(client, 1802, 100, "H07_10 负转矩限制", axis) and
                self.write_checked(client, 512, 0, "H02_00 速度模式", axis) and
                self.write_checked(client, 770, 1, "H03_02 启用伺服", axis) and
                self.write_checked(client, 771, 0, "H03_03 正逻辑", axis) and
                self.write_checked(client, 1538, 0, "H06_02 内部速度控制", axis) and
                self.write_checked(client, 1539, SPEED_RPM & 0xFFFF, "H06_03 速度", axis) and
                self.write_checked(client, 1541, 200, "H06_05 加速时间", axis) and
                self.write_checked(client, 1542, 200, "H06_06 减速时间", axis)
            )
            if not ok:
                result = "❌ 参数设置失败"
                self.results[axis] = result
                return

            self.write_u16(client, 771, 1)
            self.get_logger().info(f"[{axis}] 🚦 开始回零")

            delta_mm_1 = float('inf')
            delta_mm_2 = float('inf')
            start_time = time.time()

            while True:
                if time.time() - start_time > TIMEOUT_SEC:
                    result = "⏰ 回零超时"
                    self.write_u16(client, 1539, 0)
                    break

                abs_pos_1 = self.read_i32(client, 2893)
                if abs_pos_1 is None:
                    continue

                h0b70 = self.read_i16(client, 2886)
                h0b71 = self.read_u32(client, 2887)
                if h0b70 is None or h0b71 is None:
                    continue

                computed_pos = h0b70 * RESOLUTION + h0b71
                delta_mm_2 = min(delta_mm_2, abs(computed_pos - zero_pos) / RESOLUTION * SCREW_PITCH)
                delta_mm_1 = min(delta_mm_1, abs(abs_pos_1 - zero_pos) / RESOLUTION * SCREW_PITCH)

                self.get_logger().info(f"[{axis}] 📍 H0B_77: {abs_pos_1}, H0B_70+71: {computed_pos}, Δ1: {delta_mm_1:.4f} mm, Δ2: {delta_mm_2:.4f} mm")

                if delta_mm_1 < TOLERANCE_MM or delta_mm_2 < TOLERANCE_MM:
                    self.get_logger().info(f"[{axis}] ✅ 回零完成")
                    self.write_u16(client, 1539, 0)
                    break

                time.sleep(0.01)

            # 恢复工作力矩
            self.write_checked(client, 1801, 200, "恢复 H07_09", axis)
            self.write_checked(client, 1802, 200, "恢复 H07_10", axis)

            result = "✅ 回零完成"
        except Exception as e:
            result = f"❌ 异常: {e}"
        finally:
            client.close()
            self.results[axis] = result
            self.get_logger().info(f"[{axis}] 🔚 任务结束")

def main(args=None):
    rclpy.init(args=args)
    node = BModuleHomeService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

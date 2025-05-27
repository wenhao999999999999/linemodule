import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pymodbus.client.sync import ModbusSerialClient
import time
import threading


class LineModuleAHomeService(Node):
    def __init__(self):
        super().__init__('a_module_home_service')

        self.srv = self.create_service(Trigger, '/line_module_a_home', self.callback)
        self.get_logger().info("🟢 四轴回零服务已启动（A模组 / 单圈绝对值编码器）")

        self.axis_ports = {
            'ax1': '/dev/line_module_ax1',
            'ax2': '/dev/line_module_ax2',
            'ay':  '/dev/line_module_ay',
            'az':  '/dev/line_module_az',
        }

        self.axis_home_speed_rpm = {
            'ax1': -600,
            'ax2': -600,
            'ay':  -600,
            'az':  -600,
        }

        self.axis_results = {}
        self.axis_connected = {}
        self.nominal_torque = {
            'homing': 100,
            'working': 200
        }

        self.zero_circle = {}
        self.screw_pitch_mm = 10.0

    def reset_axis(self, port, axis_name):
        connected = False
        try:
            client = ModbusSerialClient(method='rtu', port=port, baudrate=57600, parity='N', stopbits=1, bytesize=8, timeout=1)
            if not client.connect():
                raise Exception("串口连接失败")
            connected = True
            self.axis_connected[axis_name] = True

            def write_u16(addr, val):
                client.write_register(addr, val, unit=1)

            def read_i16(addr):
                rr = client.read_holding_registers(addr, 1, unit=1)
                if rr.isError() or not hasattr(rr, 'registers'):
                    return None
                raw = rr.registers[0]
                return raw - 0x10000 if raw >= 0x8000 else raw

            def read_i32(addr):
                rr = client.read_holding_registers(addr, 2, unit=1)
                if rr.isError() or not hasattr(rr, 'registers'):
                    return None
                low, high = rr.registers[0], rr.registers[1]
                value = (high << 16) | low
                if value >= 0x80000000:
                    value -= 0x100000000
                return value

            def read_u16(addr):
                rr = client.read_holding_registers(addr, 1, unit=1)
                if rr.isError() or not hasattr(rr, 'registers'):
                    return None
                return rr.registers[0]

            def read_u32(addr):
                rr = client.read_holding_registers(addr, 2, unit=1)
                if rr.isError() or not hasattr(rr, 'registers'):
                    return None
                low, high = rr.registers[0], rr.registers[1]
                return (high << 16) | low

            def get_circle_fraction():
                h0b70 = read_i16(2886)
                h0b71 = read_u32(2887)
                if h0b70 is None or h0b71 is None:
                    return None
                return h0b70 + h0b71 / 131072

            def get_position_mm():
                circle = get_circle_fraction()
                if circle is None or axis_name not in self.zero_circle:
                    return 0.0
                return (circle - self.zero_circle[axis_name]) * self.screw_pitch_mm

            def write_checked(addr, val, name):
                write_u16(addr, val)
                time.sleep(0.02)
                rr = client.read_holding_registers(addr, 1, unit=1)
                if rr.isError() or rr.registers[0] != (val & 0xFFFF):
                    self.get_logger().error(f"[{axis_name}] ❌ 写入失败: {name}")
                    return False
                self.get_logger().info(f"[{axis_name}] ✅ 设置成功: {name} = {val}")
                return True

            write_u16(773, 1)
            time.sleep(0.1)
            write_u16(773, 0)
            self.get_logger().info(f"[{axis_name}] 🧯 故障清除完成")
            write_u16(771, 0)
            time.sleep(0.1)

            rpm = self.axis_home_speed_rpm[axis_name]
            ok = (
                write_checked(1801, self.nominal_torque['homing'], f"H07_09 = {self.nominal_torque['homing']} → 电机{self.nominal_torque['homing'] * 0.001}倍正内部转矩") and
                write_checked(1802, self.nominal_torque['homing'], f"H07_10 = {self.nominal_torque['homing']} → 电机{self.nominal_torque['homing'] * 0.001}倍负内部转矩") and
                write_checked(512, 0, "H02_00 = 0 → 速度模式") and
                write_checked(770, 1, "H03_02 = 1 → DI1伺服使能") and
                write_checked(771, 0, "H03_03 = 0 → DI1正逻辑") and
                write_checked(1024, 19, "H04_00 = 19 → DO1速度到达") and
                write_checked(1025, 0, "H04_01 = 0 → DO1正逻辑") and
                write_checked(1538, 0, "H06_02 = 0 → 内部速度指令") and
                write_checked(1539, rpm & 0xFFFF, f"H06_03 = {rpm} rpm") and
                write_checked(1541, 300, "H06_05 = 30 ms 加速") and
                write_checked(1542, 300, "H06_06 = 30 ms 减速")
            )
            if not ok:
                self.axis_results[axis_name] = "❌ 参数设置失败"
                return

            write_checked(771, 1, "H03_03 = 1 → 启用伺服")
            self.get_logger().info(f"[{axis_name}] 🚦 启动回零流程（四判据监测中）")
            time.sleep(1)

            prev_curr = prev_torque = prev_speed = None

            sample_ready = False
            speed_window = []
            STABLE_LEN = 10
            SPEED_THRESHOLD = 10

            while True:
                speed = read_i16(2816)
                curr = read_i16(2840) * 0.01
                torque = read_i16(2818) * 0.1
                triggered = False

                if abs(torque + self.nominal_torque['homing'] * 0.1) <= 0.01:
                    self.get_logger().info(f"[{axis_name}] ⏹ 原点回零判据触发：转矩满载 {torque} %")
                    triggered = True

                if triggered:
                    break

                self.get_logger().info(f"[{axis_name}] 当前转速：{speed:.1f}rpm")
                self.get_logger().info(f"[{axis_name}] 当前电流：{curr:.2f}A")
                self.get_logger().info(f"[{axis_name}] 当前转矩：{torque:.2f}%")

                if not sample_ready:
                    speed_window.append(speed)
                    if len(speed_window) > STABLE_LEN:
                        speed_window.pop(0)

                    if len(speed_window) == STABLE_LEN and max(speed_window) - min(speed_window) < SPEED_THRESHOLD:
                        prev_speed = speed
                        prev_torque = torque
                        prev_curr = curr
                        sample_ready = True
                        self.get_logger().info(f"[{axis_name}] 🎯 采样完成：speed={prev_speed:.1f}rpm, torque={prev_torque:.1f}%, curr={prev_curr:.2f}A")
                    else:
                        time.sleep(0.01)
                        continue

                def is_significant_change(current, previous, threshold_ratio):
                    if current is None or previous is None or abs(previous) < 1e-6:
                        return False
                    return abs((current - previous) / previous) > threshold_ratio

                if is_significant_change(torque, prev_torque, 4.5):
                    self.get_logger().info(f"[{axis_name}] ⏹ 判据1触发：转矩突变 {prev_torque:.1f} → {torque:.1f} %")
                    triggered = True

                if is_significant_change(curr, prev_curr, 5.0):
                    self.get_logger().info(f"[{axis_name}] ⏹ 判据2触发：电流突变 {prev_curr:.2f} → {curr:.2f} A")
                    triggered = True

                if abs(speed) <= abs(prev_speed * 0.90):
                    self.get_logger().info(f"[{axis_name}] ⏹ 判据3触发：速度过慢 speed={speed:.1f} rpm")
                    triggered = True

                if triggered:
                    break

                time.sleep(0.01)

            write_u16(1539, 15)
            time.sleep(1.5)
            write_u16(1539, 0)

            curr_circle = get_circle_fraction()
            if curr_circle is not None:
                self.zero_circle[axis_name] = curr_circle

            client.close()

            self.axis_results[axis_name] = f"✅ 回零成功：A模组轴 {axis_name.upper()}"

            write_checked(1801, self.nominal_torque['working'], f"H07_09 = {self.nominal_torque['working']} → 电机{self.nominal_torque['working'] * 0.001}倍正内部转矩")
            write_checked(1802, self.nominal_torque['working'], f"H07_10 = {self.nominal_torque['working']} → 电机{self.nominal_torque['working'] * 0.001}倍负内部转矩")

            self.get_logger().info(f"[{axis_name}] ✅ 回零逻辑执行完毕")

        except Exception as e:
            self.get_logger().error(f"[{axis_name}] ❌ 异常: {e}")
            self.axis_results[axis_name] = f"❌ 失败: {e}"
            if not connected:
                self.axis_connected[axis_name] = False

    def callback(self, request, response):
        self.axis_results.clear()
        self.axis_connected.clear()
        threads = []

        for axis_name, port in self.axis_ports.items():
            t = threading.Thread(target=self.reset_axis, args=(port, axis_name), daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        summary = "\n".join(f"[{k}] {v}" for k, v in self.axis_results.items())
        response.success = all("✅" in self.axis_results[k] for k, connected in self.axis_connected.items() if connected)
        response.message = f"A模组四轴回零完成：\n{summary}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LineModuleAHomeService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

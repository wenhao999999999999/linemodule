#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymodbus.client.sync import ModbusSerialClient
import time

class SpeedMoveTest(Node):
    def __init__(self):
        super().__init__('speed_move_test')

        self.declare_parameter("mm_target", 50.0)
        self.declare_parameter("direction", 1)
        self.mm_target = self.get_parameter("mm_target").get_parameter_value().double_value
        self.direction = self.get_parameter("direction").get_parameter_value().integer_value

        self.port = '/dev/line_module_az'
        self.client = ModbusSerialClient(method='rtu', port=self.port, baudrate=57600, timeout=1)
        assert self.client.connect(), "❌ 无法连接伺服驱动器"
        self.get_logger().info("✅ Modbus串口连接成功")

        self.screw_pitch_mm = 10.0
        self.encoder_resolution = 131072

        self.run_test()

    def read_i16(self, addr):
        rr = self.client.read_holding_registers(addr, 1, unit=1)
        if rr.isError() or not hasattr(rr, 'registers'):
            return None
        raw = rr.registers[0]
        return raw - 0x10000 if raw >= 0x8000 else raw

    def read_i32(self, addr):
        rr = self.client.read_holding_registers(addr, 2, unit=1)
        if rr.isError() or not hasattr(rr, 'registers'):
            return None
        raw = (rr.registers[0] << 16) | rr.registers[1]
        return raw - 0x100000000 if raw & 0x80000000 else raw

    def read_safe_u31(self, addr):
        """读取H0B_71的安全方法，屏蔽最高位，最大2147483647"""
        rr = self.client.read_holding_registers(addr, 2, unit=1)
        if rr.isError() or not hasattr(rr, 'registers'):
            return None
        raw = (rr.registers[0] << 16) | rr.registers[1]
        return raw & 0x7FFFFFFF

    def convert_incircle_to_mm(self, in_circle_pos):
        return (in_circle_pos / 2147483647) * self.screw_pitch_mm

    def write_speed(self, rpm):
        rpm16 = rpm & 0xFFFF
        self.client.write_register(1539, rpm16, unit=1)

    def run_test(self):
        addr_circle = 2886
        addr_abspos = 2823
        addr_incircle = 2887

        self.client.write_register(771, 0, unit=1)
        self.client.write_register(773, 1, unit=1)
        time.sleep(0.1)
        self.client.write_register(773, 0, unit=1)
        self.get_logger().info("🧯 已清除报警")

        self.client.write_register(512, 0, unit=1)
        self.client.write_register(1538, 0, unit=1)
        self.get_logger().info("⚙️ 设置速度模式完成")

        time.sleep(1)
        self.client.write_register(771, 1, unit=1)
        self.get_logger().info("⚙️ 已重新使能伺服")

        circle_before = self.read_i16(addr_circle)
        abspos_before = self.read_i32(addr_abspos)
        self.get_logger().info(f"🚦 初始圈数 H0B_70：{circle_before}")
        self.get_logger().info(f"📍 初始绝对位置 H0B_07：{abspos_before}")

        rpm_base = 150
        rpm = -self.direction * rpm_base
        direction_text = "正向" if self.direction > 0 else "负向"
        self.write_speed(rpm)
        self.get_logger().info(f"➡️ 开始{direction_text}以 {abs(rpm)} rpm 匀速运行，目标 {self.mm_target:.2f} mm")

        circle_target = self.mm_target / self.screw_pitch_mm
        speed_rps = abs(rpm) / 60.0
        duration = circle_target / speed_rps
        self.get_logger().info(f"⏱️ 预估运行时间：{duration:.2f} 秒")

        start_time = time.time()
        while time.time() - start_time < duration:
            current_circle = self.read_i16(addr_circle)
            in_circle_pos = self.read_safe_u31(addr_incircle)
            mm_in_circle = self.convert_incircle_to_mm(in_circle_pos)
            self.get_logger().info(
                f"🔄 当前圈数：{current_circle}，H0B_71：{in_circle_pos}，≈ 圈内位移：{mm_in_circle:.4f} mm"
            )
            time.sleep(0.1)

        self.write_speed(0)
        self.get_logger().info("🛑 已停止")

        circle_after = self.read_i16(addr_circle)
        abspos_after = self.read_i32(addr_abspos)
        self.get_logger().info(f"✅ 结束圈数 H0B_70：{circle_after}")
        self.get_logger().info(f"📍 结束绝对位置 H0B_07：{abspos_after}")

        delta_circle = circle_after - circle_before
        delta_abs = abspos_after - abspos_before
        delta_mm = (delta_abs / self.encoder_resolution) * self.screw_pitch_mm

        self.get_logger().info(f"📐 理论圈数：{circle_target:.2f}，实际圈数：{delta_circle}")
        self.get_logger().info(f"📏 理论位移：{self.mm_target:.2f} mm，脉冲变化：{delta_abs}，换算位移：{delta_mm:.2f} mm")

        self.client.close()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedMoveTest()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

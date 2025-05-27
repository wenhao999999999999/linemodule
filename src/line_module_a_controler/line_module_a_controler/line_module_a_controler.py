import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from pymodbus.client.sync import ModbusSerialClient
import threading
import time
from collections import defaultdict

class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(-float('inf'), float('inf')), integral_limits=(-float('inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

        self.min_output, self.max_output = output_limits
        self.min_integral, self.max_integral = integral_limits

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        derivative = (error - self.prev_error) / self.dt
        self.integral += error * self.dt
        self.integral = max(min(self.integral, self.max_integral), self.min_integral)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_output), self.min_output)
        self.prev_error = error
        return output

class LineModuleAController(Node):
    def __init__(self):
        super().__init__('line_module_a_controller')

        self.screw_pitch_mm = 10.0
        self.speed_max = 1200
        self.speed_gain = 15

        self.lock_ax = threading.Lock()
        self.lock_ay = threading.Lock()
        self.lock_az = threading.Lock()

        self.axis_ports = {
            'ax1': '/dev/line_module_ax1',
            'ax2': '/dev/line_module_ax2',
            'ay':  '/dev/line_module_ay',
            'az':  '/dev/line_module_az',
        }

        self.axis_clients = {}
        self.axis_connected = {}
        self.zero_circle = {}
        self.target_mm = {'ax': None, 'ay': None, 'az': None}
        self.last_valid_circle = {}

        self.init_modbus_clients()

        if not self.call_home_service_sync():
            raise RuntimeError("回零失败")

        for axis in self.axis_ports:
            circle = self.get_circle_fraction(axis)
            if circle is None:
                raise RuntimeError(f"读取编码器圈数失败: {axis}")
            self.zero_circle[axis] = circle
            self.last_valid_circle[axis] = circle
            self.get_logger().info(f"[Zero] {axis} 原点圈数 = {circle:.5f}")

        self.create_subscription(Float32MultiArray, '/target_position_array', self.listener_callback_all, 10)
        self.pub_position_string = self.create_publisher(String, '/line_module_A/position_mm', 10)
        self.create_timer(0.2, self.publish_position_all)

        self.pid_controllers = defaultdict(lambda: PIDController(
            kp=8.0, ki=0.3, kd=0.1, dt=0.01,
            output_limits=(-self.speed_max, self.speed_max),
            integral_limits=(-100, 100)
        ))
        threading.Thread(target=self.control_loop_ax, daemon=True).start()
        threading.Thread(target=self.control_loop, args=('ay',), daemon=True).start()
        threading.Thread(target=self.control_loop, args=('az',), daemon=True).start()

    def init_modbus_clients(self):
        for axis, port in self.axis_ports.items():
            client = ModbusSerialClient(method='rtu', port=port, baudrate=57600, timeout=1)
            if client.connect():
                self.axis_clients[axis] = client
                self.axis_connected[axis] = True
                self.get_logger().info(f"✅ 成功连接 {axis} → {port}")
            else:
                self.axis_clients[axis] = None
                self.axis_connected[axis] = False
                self.get_logger().error(f"❌ 串口连接失败 {axis} → {port}")

        if not any(self.axis_connected.values()):
            raise RuntimeError("❌ 所有串口连接失败")

    def call_home_service_sync(self):
        client = self.create_client(Trigger, '/line_module_home_service')
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > 15:
                return False
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            return future.result().success
        except Exception:
            return False

    def get_circle_fraction(self, axis):
        client = self.axis_clients[axis]

        def read_i16(addr):
            rr = client.read_holding_registers(addr, 1, unit=1)
            if rr.isError() or not hasattr(rr, 'registers'):
                return None
            raw = rr.registers[0]
            return raw - 0x10000 if raw >= 0x8000 else raw

        def read_u31(addr):
            rr = client.read_holding_registers(addr, 2, unit=1)
            if rr.isError() or not hasattr(rr, 'registers'):
                return None
            return (rr.registers[0] << 16 | rr.registers[1]) & 0x7FFFFFFF

        h0b70 = read_i16(2886)
        h0b71 = read_u31(2887)
        if h0b70 is None or h0b71 is None:
            return None

        circle = h0b70 + h0b71 / 2147483647
        self.last_valid_circle[axis] = circle
        return circle

    def get_position_mm(self, axis):
        if axis not in self.zero_circle:
            self.get_logger().error(f"❌ {axis} 无原点记录")
            return 0.0
        circle = self.get_circle_fraction(axis)
        if circle is None:
            return 0.0
        return (circle - self.zero_circle[axis]) * self.screw_pitch_mm

    def write_speed(self, axis, rpm):
        client = self.axis_clients[axis]
        rpm16 = rpm & 0xFFFF
        client.write_register(1539, rpm16, unit=1)

    def control_loop_ax(self):
        while rclpy.ok():
            with self.lock_ax:
                target_mm = self.target_mm['ax']
            if target_mm is None:
                time.sleep(0.01)
                continue
            current_circle = self.get_circle_fraction('ax1')
            target_circle = target_mm / self.screw_pitch_mm + self.zero_circle['ax1']
            self.get_logger().info(f"[AX] 当前圈数: {current_circle:.5f}, 目标圈数: {target_circle:.5f}")
            pos_mm = self.get_position_mm('ax1')
            error = pos_mm - target_mm
            if abs(error) <= 0.1:
                self.write_speed('ax1', 0)
                self.write_speed('ax2', 0)
                with self.lock_ax:
                    self.target_mm['ax'] = None
                    self.pid_controllers['ax'].reset()
                continue
            speed = int(self.pid_controllers['ax'].compute(error))
            self.write_speed('ax1', speed)
            self.write_speed('ax2', speed)
            time.sleep(0.01)

    def control_loop(self, axis):
        lock = getattr(self, f'lock_{axis}')
        while rclpy.ok():
            with lock:
                target_mm = self.target_mm[axis]
            if target_mm is None:
                time.sleep(0.01)
                continue
            current_circle = self.get_circle_fraction(axis)
            target_circle = target_mm / self.screw_pitch_mm + self.zero_circle[axis]
            self.get_logger().info(f"[{axis.upper()}] 当前圈数: {current_circle:.5f}, 目标圈数: {target_circle:.5f}")
            pos_mm = self.get_position_mm(axis)
            error = pos_mm - target_mm
            if abs(error) <= 0.1:
                self.write_speed(axis, 0)
                with lock:
                    self.target_mm[axis] = None
                self.pid_controllers[axis].reset()
                continue
            speed = int(self.pid_controllers[axis].compute(error))
            self.write_speed(axis, speed)
            time.sleep(0.01)

    def listener_callback_all(self, msg):
        if len(msg.data) >= 3:
            with self.lock_ax:
                self.target_mm['ax'] = max(min(0.0, msg.data[0]), -900.0)
            with self.lock_ay:
                self.target_mm['ay'] = max(min(0.0, msg.data[1]), -950.0)
            with self.lock_az:
                self.target_mm['az'] = max(min(0.0, msg.data[2]), -300.0)

    def publish_position_all(self):
        x = (self.get_position_mm('ax1') + self.get_position_mm('ax2')) / 2.0
        y = self.get_position_mm('ay')
        z = self.get_position_mm('az')
        msg = String()
        msg.data = f"模组A位置：[x={x:.2f}, y={y:.2f}, z={z:.2f}] mm"
        self.pub_position_string.publish(msg)

    def close(self):
        for client in self.axis_clients.values():
            if client:
                client.close()

def main(args=None):
    rclpy.init(args=args)
    node = LineModuleAController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

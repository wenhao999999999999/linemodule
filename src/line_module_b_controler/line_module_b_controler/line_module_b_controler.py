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

class LineModuleBController(Node):
    def __init__(self):
        super().__init__('line_module_b_controller')

        self.screw_pitch_mm = 10.0
        self.speed_max = 1200

        self.lock_bx = threading.Lock()
        self.lock_by = threading.Lock()
        self.lock_bz = threading.Lock()

        self.axis_ports = {
            'bx1': '/dev/line_module_bx1',
            'bx2': '/dev/line_module_bx2',
            'by':  '/dev/line_module_by',
            'bz':  '/dev/line_module_bz',
        }

        self.zero_position = {}
        self.axis_clients = {}
        self.target_mm = {'bx': None, 'by': None, 'bz': None}

        self.init_modbus_clients()

        if not self.call_home_service_sync():
            self.get_logger().error("❌ 回零服务调用失败，退出位置控制")
            raise RuntimeError("B模组回零失败")

        for axis in self.axis_ports:
            pos = self.read_abs_position(axis)
            if pos is not None:
                self.zero_position[axis] = pos
                self.get_logger().info(f"[Zero] {axis} 原点位置 = {pos}")
            else:
                self.get_logger().error(f"[Zero] ❌ 无法读取 {axis} 零点位置")
                raise RuntimeError("编码器读数失败")

        self.create_subscription(Float32MultiArray, '/target_position_array', self.listener_callback_all, 10)
        self.pub_position_string = self.create_publisher(String, '/line_module_B/position_mm', 10)
        self.create_timer(0.2, self.publish_position_all)

        self.pid_controllers = defaultdict(lambda: PIDController(
            kp=8.0, ki=0.3, kd=0.1, dt=0.01,
            output_limits=(-self.speed_max, self.speed_max),
            integral_limits=(-100, 100)
        ))

        threading.Thread(target=self.control_loop_bx, daemon=True).start()
        threading.Thread(target=self.control_loop, args=('by',), daemon=True).start()
        threading.Thread(target=self.control_loop, args=('bz',), daemon=True).start()

    def call_home_service_sync(self):
        client = self.create_client(Trigger, '/line_module_b_home')
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

    def init_modbus_clients(self):
        for axis, port in self.axis_ports.items():
            client = ModbusSerialClient(method='rtu', port=port, baudrate=57600, timeout=1)
            if client.connect():
                self.axis_clients[axis] = client
                self.get_logger().info(f"✅ 成功连接 {axis} → {port}")
            else:
                self.axis_clients[axis] = None
                self.get_logger().error(f"❌ 串口连接失败 {axis} → {port}")

    def read_abs_position(self, axis):
        client = self.axis_clients.get(axis)
        if client is None:
            return None
        rr = client.read_holding_registers(2893, 2, unit=1)
        if rr.isError() or not hasattr(rr, 'registers'):
            return None
        low, high = rr.registers[0], rr.registers[1]
        value = (high << 16) | low
        if value >= 0x80000000:
            value -= 0x100000000
        return value

    def get_position_mm(self, axis):
        abs_pos = self.read_abs_position(axis)
        if abs_pos is None:
            return 0.0
        zero = self.zero_position.get(axis, 0)
        return (abs_pos - zero) / 131072 * self.screw_pitch_mm

    def write_speed(self, axis, rpm):
        client = self.axis_clients.get(axis)
        if client:
            rpm16 = rpm & 0xFFFF
            client.write_register(1539, rpm16, unit=1)

    def control_loop_bx(self):
        while rclpy.ok():
            with self.lock_bx:
                target_mm = self.target_mm['bx']
            if target_mm is None:
                time.sleep(0.01)
                continue
            pos1 = self.get_position_mm('bx1')
            pos2 = self.get_position_mm('bx2')
            pos_mm = (pos1 + pos2) / 2.0
            error = pos_mm - target_mm
            if abs(error) <= 0.1:
                self.write_speed('bx1', 0)
                self.write_speed('bx2', 0)
                with self.lock_bx:
                    self.target_mm['bx'] = None
                self.pid_controllers['bx'].reset()
                continue
            speed = int(self.pid_controllers['bx'].compute(error))
            self.write_speed('bx1', speed)
            self.write_speed('bx2', speed)
            time.sleep(0.01)

    def control_loop(self, axis):
        lock = getattr(self, f'lock_{axis}')
        while rclpy.ok():
            with lock:
                target_mm = self.target_mm[axis]
            if target_mm is None:
                time.sleep(0.01)
                continue
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
        if len(msg.data) >= 6:
            with self.lock_bx:
                self.target_mm['bx'] = max(min(0.0, msg.data[3]), -900.0)
            with self.lock_by:
                self.target_mm['by'] = max(min(0.0, msg.data[4]), -950.0)
            with self.lock_bz:
                self.target_mm['bz'] = max(min(0.0, msg.data[5]), -300.0)

    def publish_position_all(self):
        x = (self.get_position_mm('bx1') + self.get_position_mm('bx2')) / 2.0
        y = self.get_position_mm('by')
        z = self.get_position_mm('bz')
        msg = String()
        msg.data = f"模组B位置：[x={x:.2f}, y={y:.2f}, z={z:.2f}] mm"
        self.pub_position_string.publish(msg)

    def close(self):
        for client in self.axis_clients.values():
            if client:
                client.close()

def main(args=None):
    rclpy.init(args=args)
    node = LineModuleBController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
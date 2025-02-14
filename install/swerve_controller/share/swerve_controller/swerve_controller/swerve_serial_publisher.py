import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import serial
import struct
import time

class SwerveSerialPublisher(Node):
    def __init__(self):
        super().__init__('swerve_serial_publisher')

        # พยายามเปิด Serial Port
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info("✅ Serial port opened successfully.")
        except serial.SerialException:
            self.get_logger().error("❌ Failed to open Serial port! Check connection.")
            self.ser = None

        # Subscribe จาก `/joint_states`
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50  # เพิ่ม queue size ให้สูงขึ้นเพื่อลดดีเลย์
        )

        # Timer สำหรับส่งข้อมูลทุก 20ms (50Hz)
        self.send_interval = 0.02  # 20ms
        self.timer = self.create_timer(self.send_interval, self.send_serial_data)

        # ตัวแปรเก็บค่าล่าสุดของ joint states
        self.latest_joint_state = None

    def joint_state_callback(self, msg):
        """ เก็บค่าล่าสุดของ `/joint_states` """
        self.latest_joint_state = msg

    def calculate_checksum(self, data):
        """ คำนวณ Checksum โดยใช้ XOR """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def process_wheel_value(self, value):
        """ ปัดเศษค่า Wheel เป็นทศนิยม 2 ตำแหน่ง และเซ็ตเป็น 0 ถ้าใกล้ 0 """
        if abs(value) < 0.10:  # ถ้าเข้าใกล้ 0 ให้เป็น 0
            return 0.0
        return round(value, 2)

    def process_steering_value(self, value):
        """ 
        ปัดเศษค่า Steering เป็นทศนิยม 2 ตำแหน่ง 
        - ถ้าค่า < -1.59 หรือ > 1.59 ให้เป็น 0
        - ถ้าเข้าใกล้ 0 ก็ให้เป็น 0 
        """
        if abs(value) < 0.01 or abs(value) > 1.59:
            return 0.0
        return round(value, 2)

    def send_serial_data(self):
        """ อ่านค่าจาก `latest_joint_state` และส่ง Serial Data """
        if self.ser is None or self.latest_joint_state is None:
            return

        joint_map = {name: pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)}
        velocity_map = {name: vel for name, vel in zip(self.latest_joint_state.name, self.latest_joint_state.velocity)}

        # Steering Position (ใช้เงื่อนไขเพิ่มเติม)
        servo1 = self.process_steering_value(joint_map.get("steering_front_right_joint", 0.0))
        servo2 = self.process_steering_value(joint_map.get("steering_front_left_joint", 0.0))
        servo3 = self.process_steering_value(joint_map.get("steering_rear_left_joint", 0.0))
        servo4 = self.process_steering_value(joint_map.get("steering_rear_right_joint", 0.0))

        # Wheel Velocity (ใช้การปัดเศษ)
        motor1 = self.process_wheel_value(velocity_map.get("wheel_front_right_joint", 0.0))
        motor2 = self.process_wheel_value(velocity_map.get("wheel_rear_left_joint", 0.0))
        motor3 = self.process_wheel_value(velocity_map.get("wheel_rear_right_joint", 0.0))
        motor4 = self.process_wheel_value(velocity_map.get("wheel_front_left_joint", 0.0))

        # สร้าง Packet Data
        data_packet = struct.pack('8f', motor1, motor2, motor3, motor4, servo1, servo2, servo3, servo4)

        # คำนวณ Checksum
        checksum = self.calculate_checksum(data_packet)

        # รวม Packet + Checksum
        final_packet = data_packet + struct.pack('B', checksum)

        # ส่งข้อมูลไปที่ Arduino Mega
        self.ser.write(final_packet)

        # แสดงค่าที่ส่งออก Terminal
        self.get_logger().info(f'Sent Data (Raw): {final_packet}')
        self.get_logger().info(f'Motor: [{motor1}, {motor2}, {motor3}, {motor4}]')
        self.get_logger().info(f'Servo: [{servo1}, {servo2}, {servo3}, {servo4}]')
        self.get_logger().info(f'Checksum: {checksum}')
        self.get_logger().info('-' * 50)  # แยกผลลัพธ์ให้อ่านง่าย

def main(args=None):
    rclpy.init(args=args)

    # ใช้ MultiThreadedExecutor เพื่อให้ Node ทำงานเร็วขึ้น
    node = SwerveSerialPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

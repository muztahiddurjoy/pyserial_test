import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

# class SerialNode(Node):
#     def __init__(self):
#         super().__init__('py_getter')
#         self.declare_parameter('port', '/dev/ttyUSB0')
#         self.declare_parameter('baudrate', 9600)

#         port = self.get_parameter('port').value
#         baudrate = self.get_parameter('baudrate').value

#         try:
#             self.serial_connection = serial.Serial(port, baudrate, timeout=1)
#             self.get_logger().info(f"Connected to {port} at {baudrate} baud.")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to connect to serial port: {e}")
#             return

#         # Example: Read data at a fixed rate
#         self.timer = self.create_timer(0.1, self.read_from_serial)

#     def read_from_serial(self):
#         try:
#             if self.serial_connection.in_waiting > 0:
#                 data = self.serial_connection.readline().decode().strip()
#                 self.get_logger().info(f"Received: {data}")
#         except Exception as e:
#             self.get_logger().error(f"Error reading from serial: {e}")

#     def __del__(self):
#         if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
#             self.serial_connection.close()
#             self.get_logger().info("Serial connection closed.")


class PySender(Node):
    def __init__(self):
        super().__init__("led_controller")
        self.declare_parameter('port','/dev/ttyACM0')
        self.declare_parameter('baudrate',9600)

        port = self.get_parameter('port').value
        
        baud = self.get_parameter('baudrate').value

        try:
            self.serial_connection = serial.Serial(port,baud,timeout=1)
            self.get_logger().info(f"Connected to {port} with baudrate {baud}")
        except serial.SerialException as e:
            self.get_logger().info("Exception while connecting: {e}")
        self.led_subscriber = self.create_subscription(String,"control_led",self.send_serial_data,10)
    
    def send_serial_data(self,msg):
        try:
            char = msg.data
            if len(char) ==1:
                self.serial_connection.write(char.encode())
                self.get_logger().info(f"Sent {char}")
        except Exception as e:
            self.get_logger().error(f"Got error while sending data tp port {e}")
    def __del__(self):
        if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Serial connection closed.")
def main(args=None):
    rclpy.init(args=args)
    node = PySender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

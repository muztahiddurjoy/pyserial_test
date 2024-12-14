import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg._twist import Twist

class RoverController(Node):
    def __init__(self):
        super().__init__("rover_controller")
        self.declare_parameter('port','/dev/ttyACM0')
        self.declare_parameter('baudrate',9600)

        port = self.get_parameter('port').value
        
        baud = self.get_parameter('baudrate').value

        try:
            self.serial_connection = serial.Serial(port,baud,timeout=1)
            self.get_logger().info(f"Connected to {port} with baudrate {baud}")
            self.timer = self.create_timer(0.1, self.read_from_serial)
        except serial.SerialException as e:
            self.get_logger().info("Exception while connecting: {e}")
        self.led_subscriber = self.create_subscription(Twist,"mt_cmd_vel",self.send_serial_data,10)
    
    def read_from_serial(self):
        try:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode().strip()
                self.get_logger().info(f"Received: {data}")
        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {e}")



    def send_serial_data(self,msg):
        try:
            throttle = msg.linear.x
            steering = msg.angular.z
            command = f"L{throttle:.2f} A{steering:.2f}\n"
            self.serial_connection.write(command.encode())
            # self.get_logger().info(f"L{throttle} A{steering}")
        except Exception as e:
            self.get_logger().error(f"Got error while sending data to port {e}")
        #     if len(char) ==1:
        #         self.serial_connection.write(char.encode())
        #         self.get_logger().info(f"Sent {char}")
        # except Exception as e:
        #     self.get_logger().error(f"Got error while sending data tp port {e}")
    def __del__(self):
        if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Serial connection closed.")
def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

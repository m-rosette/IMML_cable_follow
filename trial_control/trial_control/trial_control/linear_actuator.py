import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from trial_control_msgs.srv import LinearActuator, CableState
import serial

from rclpy.executors import MultiThreadedExecutor


class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        self.get_logger().info("Starting linear actuator control")

        # Assuming the cable starts seated (1)
        self.cable_state = 1

        # Create publisher for cable status
        self.cable_status_pub = self.create_publisher(Int16, 'cable_status', 1)
        self.cable_status_timer = self.create_timer(0.1, self.cable_status_pub_callback)

        # Create services
        self.linear_actuator_srv = self.create_service(LinearActuator, 'linear_actuator_control', self.linear_actuator_callback)
        self.cable_state_srv = self.create_service(CableState, 'cable_state', self.cable_state_callback)

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'
        self.baudrate = 115200

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)


    def linear_actuator_callback(self, request, response):
        self.get_logger().info('Received movement request')
        try:
            # Update response
            response.success = True
            move_dir = request.move_direction
            self.get_logger().info(f"Requesting movement to {request.move_direction}")

            # Send command to arduino
            self.arduino.write(move_dir.to_bytes(2, byteorder='little'))

        except ValueError:
            response.success = False

        return response
    
    def cable_state_callback(self, request, response):
        # Get cable seatment status
        cable_status = self.arduino.readline().decode().split('\r')
        self.get_logger().info(cable_status[0])
        if cable_status[0] != "":
            self.cable_state = int(cable_status[0])  # Convert to integer

        self.get_logger().info(f"Cable state: {self.cable_state}")

        response.state = self.cable_state
            
        return response
    
    def cable_status_pub_callback(self):
        msg = Int16()
        msg.data = self.cable_state
        self.cable_status_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    linear_actuator = LinearActuatorControl()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(linear_actuator, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
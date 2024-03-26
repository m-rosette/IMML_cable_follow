import rclpy
from rclpy.node import Node
from trial_control_msgs.srv import LinearActuator
from trial_control_msgs.msg import CableState
import serial


class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        self.get_logger().info("Starting linear actuator control")
        self.srv = self.create_service(LinearActuator, 'linear_actuator_control', self.linear_actuator_callback)

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'   # Change this to the correct port
        self.baudrate = 115200

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)

        # Create a timer to periodically check button status
        self.timer_period = 0.1  # 100 milliseconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Create a publisher for button state
        self.button_pub = self.create_publisher(CableState, 'cable_state', 10)

    def linear_actuator_callback(self, request, response):
        self.get_logger().info('Received movement request')
        try:
            # Update response
            response.success = True
            self.get_logger().info(f"Requesting movement to {request.move_direction}")

            # Send command to arduino
            self.arduino.write(str(request.move_direction).encode())

        except ValueError:
            response.success = False

        return response
    
    def timer_callback(self):
        # Get cable seatment status
        cable_status = self.arduino.readline().decode().strip()

        # Publish cable seatment state
        msg = CableState()
        msg.button_state = cable_status
        self.button_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    linear_actuator = LinearActuatorControl()
    rclpy.spin(linear_actuator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
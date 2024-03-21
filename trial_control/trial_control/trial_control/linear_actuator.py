import rclpy
from rclpy.node import Node
from gripper_msgs.srv import LinearActuator
import serial


class MoveMe(Node):
    def __init__(self):
        super().__init__('actuator_linear')
        self.get_logger().info("Starting linear actuator control")
        self.srv = self.create_service(LinearActuator, 'input_number', self.input_number_callback)

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'   # Change this to the correct port
        self.baudrate = 115200

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)

    def input_number_callback(self, request, response):
        self.get_logger().info('Received input_number request')
        try:
            # Update response
            response.success = True
            self.get_logger().info(f"Got position value: {request.location_goal}")

            # Send command to arduino
            self.arduino.write(str(request.location_goal).encode())

        except ValueError:
            response.success = False

        return response
    

def main(args=None):
    rclpy.init(args=args)
    actuator_linear= MoveMe()
    rclpy.spin(actuator_linear)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from trial_control_msgs.srv import LinearActuator, CableState
import serial

from rclpy.executors import MultiThreadedExecutor


class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        self.get_logger().info("Starting linear actuator control")
        self.srv = self.create_service(LinearActuator, 'linear_actuator_control', self.linear_actuator_callback)

        self.cable_state_srv = self.create_service(CableState, 'cable_state', self.cable_state_callback)

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'   # Change this to the correct port
        self.baudrate = 115200

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)

        self.cable_state = None

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
        

def main(args=None):
    rclpy.init(args=args)
    linear_actuator = LinearActuatorControl()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(linear_actuator, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
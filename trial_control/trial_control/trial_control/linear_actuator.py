import rclpy
from rclpy.node import Node
from trial_control_msgs.srv import LinearActuator, CableState
# from trial_control_msgs.msg import CableState
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

        # # Create a publisher for button state
        # self.cable_state_pub = self.create_publisher(CableState, 'cable_state', 1)

        # self.run()

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
    
    def cable_state_callback(self, request, response):
        if self.arduino.in_waiting > 0:
            # Get cable seatment status
            cable_status = self.arduino.readline().decode().split('\r')

            self.get_logger().info(f"Cable state: {cable_status[0]}")

            response.state = cable_status[0]
    
    # def run(self):
    #     while rclpy.ok():
    #         if self.arduino.in_waiting > 0:
    #            # Get cable seatment status
    #             cable_status = self.arduino.readline().decode().split('\r')

    #             self.get_logger().info(f"Cable state: {cable_status[0]}")

    #             # Publish cable seatment state
    #             msg = CableState()
    #             msg.cable_state = cable_status[0]
    #             self.cable_state_pub.publish(msg) 
        

def main(args=None):
    rclpy.init(args=args)
    linear_actuator = LinearActuatorControl()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(linear_actuator, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
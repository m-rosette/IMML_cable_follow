import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int64
from trial_control_msgs.srv import LinearActuator, CableState
import serial
import threading

class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        self.get_logger().info("Starting linear actuator control")

        # Assuming the cable starts seated (1)
        self.cable_state = 1
        self.current_stepper_pos = 0

        # Create publisher for cable status
        self.cable_status_pub = self.create_publisher(Int16, 'cable_status', 1)
        self.cable_status_timer = self.create_timer(0.1, self.cable_status_pub_callback)

        self.stepper_pos_pub = self.create_publisher(Int64, 'stepper_pos', 1)
        self.stepper_pos_timer = self.create_timer(0.1, self.stepper_pos_pub_callback)

        # Create services
        self.linear_actuator_srv = self.create_service(LinearActuator, 'linear_actuator_control', self.linear_actuator_callback)
        # self.cable_state_srv = self.create_service(CableState, 'cable_state', self.cable_state_callback)
        self.cable_pull_jig_state = self.create_service(CableState, 'cable_pull_jig_state', self.cable_pull_jig_state_callback)

        # Define Arduino serial port and baud rate
        self.arduino_port = '/dev/ttyACM1'
        self.baudrate = 115200

        # Open the serial connection to Arduino
        self.arduino = serial.Serial(self.arduino_port, self.baudrate, timeout=1)

        # Start a separate thread for reading data from the serial port
        self.serial_thread = threading.Thread(target=self.serial_read_thread)
        self.serial_thread.daemon = True  # Set as daemon so it terminates with main thread
        self.serial_thread.start()

    def serial_read_thread(self):
        while rclpy.ok():
            # Get cable seatment status
            line = self.arduino.readline().strip()
            cable_pull_jig_status = line.split(b',')

            # Check if the received line contains valid data
            if len(cable_pull_jig_status) == 2 and cable_pull_jig_status[0]:
                cable_status_coded, current_stepper_pos_coded = cable_pull_jig_status
                self.cable_state = int(cable_status_coded.decode())
                self.current_stepper_pos = int(current_stepper_pos_coded.decode())

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
    
    # def cable_state_callback(self, request, response):
    #     response.state = self.cable_state
    #     return response
    
    def cable_pull_jig_state_callback(self, request, response):
        response.cable_state = self.cable_state
        response.stepper_position = self.current_stepper_pos
        return response
    
    def cable_status_pub_callback(self):
        msg = Int16()
        msg.data = self.cable_state
        self.cable_status_pub.publish(msg)

    def stepper_pos_pub_callback(self):
        msg = Int64()
        msg.data = self.current_stepper_pos
        self.stepper_pos_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    linear_actuator = LinearActuatorControl()
    rclpy.spin(linear_actuator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

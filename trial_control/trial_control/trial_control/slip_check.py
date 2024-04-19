import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from trial_control_msgs.srv import TactileSlip
from sensor_interfaces.msg import SensorState


class TactileSlipCheck(Node):
    def __init__(self):
        super().__init__('tactile_slip_check')
        self.get_logger().info("Starting tactile slip check")

        self.tactile_0_slipstate = []
        self.tactile_1_slipstate = []   

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = self.create_subscription(SensorState, 'hub_0/sensor_0', self.tactile_0_callback, 1)
        self.tactile_1_sub = self.create_subscription(SensorState, 'hub_0/sensor_1', self.tactile_1_callback, 1)

        self.tactile_slip_srv = self.create_service(TactileSlip, 'tactile_slip', self.tactile_slip_callback)

    def tactile_0_callback(self, tac_msg):
        self.tactile_0_slipstate.clear()  # Clear the list before appending new values
        for i in range(9):
            self.tactile_0_slipstate.append(tac_msg.pillars[i].slip_state)

    def tactile_1_callback(self, tac_msg):
        self.tactile_1_slipstate.clear()  # Clear the list before appending new values
        for i in range(9):
            self.tactile_1_slipstate.append(tac_msg.pillars[i].slip_state)

    def tactile_slip_callback(self, request, response):
        self.get_logger().info('Received slip check request')
        try:
            # Update response
            response.tactile0_slip = self.tactile_0_slipstate
            response.tactile1_slip = self.tactile_1_slipstate

        except ValueError:
            pass

        return response
        

def main(args=None):
    rclpy.init(args=args)
    tactile_slip = TactileSlipCheck()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(tactile_slip, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
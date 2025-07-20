from custom_interfaces.srv import Control                                                     
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('control_client_async')
        
        self.cli = self.create_client(Control, 'control')     
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Control.Request()                                

    def send_request(self):
        self.req.joint_name = str(sys.argv[1])
        self.req.goal_theta = float(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main():
    rclpy.init()

    control_client = MinimalClientAsync()
    future = control_client.send_request()
    # rclpy.spin_until_future_complete(control_client, future)
    # response = future.result()
    # control_client.get_logger().info(
    #     'Resultant effort: for joint: %s  and goal theta: %f is %f' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.effort))


    while rclpy.ok():
        rclpy.spin_once(control_client)
        if control_client.future.done():
            try:
                response = control_client.future.result()
            except Exception as e:
                control_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                control_client.get_logger().info(
                    'Resultant effort for joint: %s and goal theta: %f is: %f' %
                    (str(sys.argv[1]), float(sys.argv[2]), float(response.effort)))
            break

    control_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

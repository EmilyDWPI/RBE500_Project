#{x: 2.0, y: 0.0, z: 0.75, quat_x: 0.0, quat_y: 0.0, quat_z: 0.0, quat_w: 0.0}


from control.srv import control                                                     
import rclpy
import math as m
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
from numpy import cos, sin, deg2rad, rad2deg, pi, sqrt, arctan2
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose 
from scipy.spatial.transform import Rotation as R


class MinimalService(Node):

    def __init__(self):
        super().__init__('control_service')
        self.srv = self.create_service(control, 'control', self.control_callback)
        self.joint_states = None  # Store latest JointState message

        # Subscribe to joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def control_callback(self, request, response):   
        # https://docs.ros.org/en/kinetic/api/gazebo_msgs/html/srv/ApplyJointEffort.html        response.
        # Lets make a program to take a difference in angle, and goes there with a PD controller
        # First we need the angle difference
        # Then we run it through our transfer function
        # Then using the /gazebo_msgs/ApplyJointEffort topic, we apply torque to the joint in the controlled fashion           
        # This should constantly apply a torque based on the difference of current theta and desired theta
        # Hence, this will be a recursive service
        Kd = 0.001  # Derivative gain
        Kp = 0.4/3  # Proportional gain
        ts = 3 # time to reach the goal
        J = 0.1 # Link inertia
        b = 1 # Link damping

        if self.joint_states is None:
            self.get_logger().warn('No joint states received yet.')
            response.effort = 0.0
            response.joint_name = request.joint_name
            return response

        # Find the index of the requested joint
        try:
            idx = self.joint_states.name.index(request.joint_name)
            current_position = self.joint_states.position[idx]
        except ValueError:
            self.get_logger().warn(f'Joint {request.joint_name} not found in joint_states.')
            response.effort = 0.0
            response.joint_name = request.joint_name
            return response
        
        error = request.theta - current_position

        self.get_logger().info(
            f'Incoming request\n Joint name: {request.joint_name} Goal Theta: {request.goal_theta} Current Theta: {current_position} Error: {error}'
        )
        response.effort = (Kp + Kd)/(J + (b+Kd) + Kp)  # Example, update as needed
        response.joint_name = request.joint_name

        print(response)
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

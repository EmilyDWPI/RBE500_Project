#{x: 2.0, y: 0.0, z: 0.75, quat_x: 0.0, quat_y: 0.0, quat_z: 0.0, quat_w: 0.0}


from custom_interfaces.srv import Control                                                     
import rclpy
import math as m
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import numpy as np
from numpy import cos, sin, deg2rad, rad2deg, pi, sqrt, arctan2
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose 
from scipy.spatial.transform import Rotation as R

from gazebo_msgs.srv import ApplyJointEffort  # Gazebo service to apply joint effort
import time


class MinimalService(Node):

    def __init__(self):
        super().__init__('control_service')
        self.srv = self.create_service(Control, 'control', self.control_callback)
        self.joint_states = None  # Store latest JointState message

        #derivative calculation variables
        self.last_error = 0.0  # Initialize last error
        self.last_time = self.get_clock().now()  # Initialize last time
        self.apply_effort_client = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')  # Gazebo effort application client
        self.timer = None  # Timer for periodic updates
        self.active_request = None  # Track the active request
        self.effort_pubs = {}  # Cache for publishers

        # Subscribe to joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.position_log = []  # Store (timestamp, position) tuples
        self.log_written = False  # Flag to avoid writing multiple times

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def control_callback(self, request, response):   

        self.active_request = request  # Store the active request

        # Check if joint_states has been received
        if self.joint_states is None:
            self.get_logger().warn('No joint states received yet.')
            response.effort = 0.0
            response.joint_name = request.joint_name
            return response

        # Find the index of the requested joint and position
        try:
            idx = self.joint_states.name.index(request.joint_name)
            current_position = self.joint_states.position[idx]
        except ValueError:
            self.get_logger().warn(f'Joint {request.joint_name} not found in joint_states.')
            response.effort = 0.0
            response.joint_name = request.joint_name
            return response
        
        # The error between the current arm theta and the goal theta
        error = self.active_request.goal_theta - current_position

        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(0.05, self.apply_pd_effort) # 20hz update of joint effort

        self.get_logger().info(
            f'Incoming request\n Joint name: {self.active_request.joint_name} Goal Theta: {self.active_request.goal_theta} Current Theta: {current_position} Error: {error}'
        )

        #response.joint_name = request.joint_name

        print(response)
        return response
    
    def get_effort_pub(self, joint_name):
        topic = f"/scara/{joint_name}_effort_controller/command"
        if topic not in self.effort_pubs:
            self.effort_pubs[topic] = self.create_publisher(Float64, topic, 10)
        return self.effort_pubs[topic]

    def apply_pd_effort(self):
        if self.active_request is None:
            return
        
        try:
            idx = self.joint_states.name.index(self.active_request.joint_name)
            current_position = self.joint_states.position[idx]
        except ValueError:
            self.get_logger().warn(f'Joint {self.active_request.joint_name} not found in joint_states.')
            return
        
        # Take current time and take change in time
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt == 0: # If there was no change in time, skip the update
            return
        
        error = self.active_request.goal_theta - current_position
        error_dot = (error - self.last_error) / dt  # Derivative of error

        # PD controller parameters. 
        if(self.active_request.joint_name == 'arm_shoulder_pan_joint'):
            Kd = 0.0256  # Derivative gain
            Kp = 0.1  # Proportional gain
            effort = Kp * error + Kd * error_dot #Can't consider the system here. The input(effort) is our controller output
        elif(self.active_request.joint_name == 'arm_elbow_pan_joint'):
            Kd = 0.14
            Kp = 0.10
            effort = Kp * error + Kd * error_dot 
        elif(self.active_request.joint_name == 'arm_wrist_lift_joint'):
            Kd = 0.0012
            Kp = 0.018
            effort = Kp * error + Kd * error_dot + (0.001 * 9.81)  # Adding a constant to counteract gravity (mass times gravity)
        else:
            self.get_logger().warn(f'Unknown joint name: {self.active_request.joint_name}')
            return
        ts = 3 # time to reach the goal
        J = 0.1 # Link inertia
        b = 1 # Link damping

        # Effort unit is in Newtons

        # Dynamically get or create the publisher for the requested joint
        pub = self.get_effort_pub(self.active_request.joint_name)
        msg = Float64()
        msg.data = effort
        pub.publish(msg)

        print(f'Applying effort: {effort} for joint: {self.active_request.joint_name} with goal theta: {self.active_request.goal_theta} and current position: {current_position}')

        
        self.last_error = error  # Update last error
        self.last_time = now  # Update last time

        if abs(error) < 0.002:
            # This can cause problems due to lack of checking variables. Lets just continue the loop for now unless direct stopping is needed.
            # This also accounts for what exterior forces may do on a joint, such as gravity for the wrist lift joint
            # self.get_logger().info(f'Joint {self.active_request.joint_name} has reached approximately the goal position [{self.active_request.goal_theta}] measured at [{current_position}]. Stopping effort application.')
            # self.timer.cancel()
            # self.active_request = None
            pass

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

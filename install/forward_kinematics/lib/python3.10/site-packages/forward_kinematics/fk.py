import rclpy
import math as m
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
from numpy import cos, sin, deg2rad, rad2deg, pi
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose 
from scipy.spatial.transform import Rotation as R
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber') #Publish Pose
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.pose_call,
            10)
        self.last_pose = None
        self.timer = self.create_timer(0.5, self.print_pose)  # 0.5 seconds

    def pose_call(self, msg):   
    
        self.joint_states = msg;
        
    #Function to Calculate Transform
        def xfrm(theta, d, a, alpha):
            Ti = np.array([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha), a*sin(theta)],
            [0,sin(alpha),cos(alpha),d],
            [0,0,0,1]])
            
            return Ti
    
    #Get Joint Positions
        q1 = self.joint_states.position[0]
        q2 = self.joint_states.position[1]
        q3 = self.joint_states.position[2] 
    
    #Set Link Lengths
        bl = 1.1
        l1 = 1
        l2 = 0.5
    
    #Apply Forward Kinematics
        T01 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,bl], [0,0,0,1]])
        T12 = xfrm(q1,0,l1,0)
        T02 = T01@T12
        T23 = xfrm(q2,0,l2,0)
        T03 = T02@T23
        T34 = xfrm(0,-q3,0,pi)
        T04 = T03@T34

    #Position        
        D = T04[:-1,3]
        
    #Rotation
        rot = R.from_matrix(T04[:3,:3])
        quat = rot.as_quat()    

    #Generate Pose
        self.pose = Pose()
        
        self.pose.position.x = D[0]
        self.pose.position.y = D[1]
        self.pose.position.z = D[2]

        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        
    #Print Pose   
        print(self.pose) 
        self.last_pose = self.pose

    def print_pose(self):
        if self.last_pose is not None:
            print(self.last_pose)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

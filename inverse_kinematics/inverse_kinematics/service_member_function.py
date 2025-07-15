#{x: 2.0, y: 0.0, z: 0.75, quat_x: 0.0, quat_y: 0.0, quat_z: 0.0, quat_w: 0.0}


from inverse_kin.srv import InvKin                                                         
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
        super().__init__('minimal_service')
        self.srv = self.create_service(InvKin, 'inv_kin', self.inv_kin_callback)      

    def inv_kin_callback(self, request, joint_val):   
    # Request consists of end-effector position: {x,y,z} and orientation: {quat_x, quat_y, quat_z, quat_w}

    #Set Link Lengths
        bl = 1.1
        l1 = 1
        l2 = .5

    # Calculate the Joint Positions
        C = (request.x**2 + request.y**2 - l1**2 - l2**2)/(2*l1*l2)
        q2 = arctan2(sqrt(1-C**2), C)
        q1 = arctan2(l2*sin(q2)*request.x + request.y*(l1+l2*cos(q2)), (l1+l2*cos(q2))*request.x - l2*sin(q2)*request.y)
        q3 = -(bl - request.z) # - to account for the z direction of the prismatic joint in gazebo
        
        joint_val.q1 = q1
        joint_val.q2 = q2
        joint_val.q3 = q3
        
    # Print Values Sent to Service                                            
        self.get_logger().info('Position Values\nx: %f y: %f z: %f\nquat_x: %f quat_y: %f quat_z: %f quat_w: %f' % (request.x, request.y, request.z, request.quat_x, request.quat_y, request.quat_z, request.quat_w))  # CHANGE

        print(joint_val)
    # Return Joint Positions
        return joint_val

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaboy/Documents/RBE500_Project/RBE500_Project/install/forward_kinematics'

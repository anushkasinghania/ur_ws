import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/ur_ws/install/Universal_Robots_ROS2_GZ_Simulation'

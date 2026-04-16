import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jeen/capstone/Self-delivery_project/raspberry_pi_node/ros2_ws/install/robot_controller'

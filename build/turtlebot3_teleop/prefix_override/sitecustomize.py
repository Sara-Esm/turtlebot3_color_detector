import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/owl/projects/turtlebot3_color_detector_ws/install/turtlebot3_teleop'

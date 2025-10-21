import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/meow/ros2_ws/install/emg_shadow_hand_bridge'

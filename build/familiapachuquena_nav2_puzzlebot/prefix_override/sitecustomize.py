import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fabrih/ros2_nav_ws/src/familiapachuquena_nav2_puzzlebot(7)/install/familiapachuquena_nav2_puzzlebot'

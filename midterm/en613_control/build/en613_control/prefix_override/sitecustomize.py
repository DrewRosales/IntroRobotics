import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/drew/repos/IntroRobotics/midterm/en613_control/install/en613_control'

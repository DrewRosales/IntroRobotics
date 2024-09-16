import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/drew/repos/IntroRobotics/hw/hw2/install/assignment2'

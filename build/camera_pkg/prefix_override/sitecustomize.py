import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yeachanokh/github/robotis-internship/src/install/camera_pkg'

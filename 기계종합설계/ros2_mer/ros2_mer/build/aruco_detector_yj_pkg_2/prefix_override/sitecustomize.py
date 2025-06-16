import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/teamleafia/ros2_mer/install/aruco_detector_yj_pkg_2'

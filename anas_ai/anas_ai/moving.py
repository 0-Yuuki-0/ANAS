import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

from std_msgs.msg import UInt64MultiArray

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# constants
rotatechange = 0.2
speedchange = 0.07
occ_bins = [-1, 0, 51, 101]
map_bg_color = 1
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle, front_angle+1, 1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
mapdata = np.array(0)
length = 5


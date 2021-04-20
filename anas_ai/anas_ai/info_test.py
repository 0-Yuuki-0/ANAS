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

class Info(Node):

    def __init__(self):
        super().__init__('position')

        # Create subscriber to publish position info
        self.publisher = self.create_publisher(
            UInt64MultiArray,
            'position',
            self.publisher,
            qos_profile_sensor_data)

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.cur_position = [0,0]

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def publisher(self):
        arr = UInt64MultiArray()
        arr.data = self.cur_position
        self.publisher_.publish(arr)

    def occ_callback(self,msg):
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        cur_pos = trans.transform.translation

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        self.cur_position = [grid_x,grid_y]
        self.get_logger().info("Update position: " + str(self.cur_position))  

def main(args=None):
    rclpy.init(args=args)

    position = Position()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    rclpy.spin(position)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
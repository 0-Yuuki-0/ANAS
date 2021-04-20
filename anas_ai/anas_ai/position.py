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

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from std_msgs.msg import UInt64MultiArray, Float64MultiArray

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

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

class Position(Node):

    def __init__(self):
        super().__init__('position')

        # Create subscriber to publish position info
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'position',
            10)

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

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self,msg):
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
            
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll_trans, pitch_trans, yaw_trans = euler_from_quaternion(
            cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)


        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        arr = Float64MultiArray()
        arr.data = [float(grid_x), float(grid_y),roll_trans, pitch_trans, yaw_trans, cur_rot.x, cur_rot.y, self.pose_x,self.pose_y,self.roll,self.pitch,self.yaw,]
        self.publisher_.publish(arr)
        self.get_logger().info('Update position: X = %i , Y = %i' % (arr.data[0],arr.data[1]))
        self.get_logger().info('Update trans roll, pitch, yaw, yaw(degree): %f , %f, %f, %f' % (arr.data[2],arr.data[3],arr.data[4],math.degrees(arr.data[4])))
        self.get_logger().info('Pose position: X = %i , Y = %i' % (arr.data[7],arr.data[8]))
        self.get_logger().info('Update roll, pitch, yaw, yaw(degree): %f , %f, %f, %f' % (arr.data[9],arr.data[10],arr.data[11],math.degrees(arr.data[11])))
        
     
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
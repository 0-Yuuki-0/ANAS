# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

from std_msgs.msg import UInt64MultiArray, Float64MultiArray

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# Dilation and pathfinding library
import cv2
import scipy.stats
from scipy.ndimage.morphology import grey_dilation, generate_binary_structure, iterate_structure

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.ida_star import IDAStarFinder
from pathfinding.finder.msp import MinimumSpanningTree

# constants
rotatechange = 0.15
speedchange = 0.07
angle_adjust = - 1/2*math.pi
occ_bins = [-1, 0, 51, 101]
map_bg_color = 1
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle, front_angle+1, 1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
mapdata = np.array(0)
length = 5

st = generate_binary_structure(2,1)
dilate = 2

# funtion to generate a set of coordinate (path) to the inout target
def path_find(self,odata,grid_x,grid_y, height, width, explore_coordinate):    
    try: 
        mod_odata = np.where(np.copy(odata)-2 == 8, 0, np.copy(odata)-2)
        
        dilated = grey_dilation(mod_odata, footprint = iterate_structure(st,dilate), mode='nearest')

        matrix = np.where((dilated > 0),0,1)   
                
        maximum = 0

        for coordinate in explore_coordinate:
            if (coordinate[0] < width and coordinate[1] <height):
                point1 = np.array((coordinate[0],coordinate[1]))
                point2 = np.array((grid_x,grid_y))
                dist = np.linalg.norm(point1 - point2)
                if (dist >= maximum and (dist >= 1.5) and matrix[coordinate[1],coordinate[0]] != 0):
                    maximum = dist
                    target = [coordinate[0],coordinate[1]]

        # print(target)                      

        grid = Grid(matrix=matrix)
        # print("grid defined")

        start = grid.node(grid_x,grid_y)
        end = grid.node(target[0],target[1])

        # print("Node defined")

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        del path[0:2]
        plt.imshow(dilated, cmap='gray', origin='lower')


        # print("path found")
        # print('operations:', runs, 'path:', path)
        # print(grid.grid_str(path=path, start=start, end=end))
        return [target,path]
    
    except Exception as e:
        print("Exception at", e)
        self.get_logger().info("No path found! Map complete")
        return


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.get_logger().info('Created publisher')

        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_trans = 0
        self.pitch_trans = 0
        self.yaw_trans = 0
        self.info = [0,0]
        self.cur_position = [0,0]
        self.trans = 0
        self.state = True
        self.cur_pos_x = 0
        self.cur_pos_y = 0
        self.left_dist = 0
        self.leftfront_dist = 0
        self.front_dist = 0
        self.rightfront_dist = 0
        self.right_dist = 0

        # create subscriber to get position
        self.position_subscription = self.create_subscription(
            Float64MultiArray,
            'position',
            self.position_callback,
            10)

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def position_callback(self,msg):
        self.cur_position = [int(msg.data[0]),int(msg.data[1])]
        self.roll_trans, self.pitch_trans, self.yaw_trans = msg.data[2], msg.data[3], msg.data[4]
        self.roll, self.pitch, self.yaw = msg.data[9], msg.data[10], msg.data[11]
        self.cur_pos_x = msg.data[5]
        self.cur_pos_y = msg.data[6]
        self.pose_x = msg.data[7]
        self.pose_y = msg.data[8]
        # self.get_logger().info('Update position: X = %i , Y = %i' % (msg.data[0],msg.data[1]))
        # self.get_logger().info('Update roll, pitch, yaw: %f , %f, %f' % (msg.data[2],msg.data[3],msg.data[4]))
        # self.get_logger().info('Pose position: X = %i , Y = %i' % (msg.data[7],msg.data[8]))
        # self.get_logger().info(str(msg.data))

    def occ_callback(self, msg):
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50,
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image
        occ_counts, edges, binnum = scipy.stats.binned_statistic(
            occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((self.cur_pos_x - map_origin.x) / map_res)
        grid_y = round(((self.cur_pos_y - map_origin.y) / map_res))

        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
        odata_pass = np.int8(binnum.reshape(msg.info.height, msg.info.width))

        # set current robot location to 0
        odata[grid_y][grid_x] = 10
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)

        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)

        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(
            img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(
            self.yaw_trans)-90, expand=True, fillcolor=map_bg_color)

        kernel = np.ones((4,4))

        edges = np.where((odata == 3),1,0)
        odata_occupied = grey_dilation(edges, footprint = iterate_structure(st,3), mode='nearest')

        test_img = cv2.Canny(odata, 1, 1)
        odata_unmapped = np.where(test_img>0,1,0)

        lines = np.where((odata_unmapped - odata_occupied != 1),0,1)

        unexplored = np.where(lines == 1)
        explore_coordinate = list(zip(unexplored[1],unexplored[0]))
        # print(explore_coordinate)

        # show the image using grayscale map
        # plt.imshow(lines, cmap='gray', origin='lower')
        # plt.imshow(edges, cmap='gray', origin='lower')
        # plt.imshow(images, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

        # np.savetxt("map.txt", odata)

        height = msg.info.height
        width = msg.info.width

        self.info = path_find(self,odata_pass,grid_x,grid_y, height, width, explore_coordinate)

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw_trans
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = rot_angle
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # self.get_logger().info('Calculate c_change')
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        # self.get_logger().info('Calculate c_change_dir')
        twist.linear.x = 0.0
        # set the direction to rotate
        # self.get_logger().info('set twist linear x')
        twist.angular.z = c_change_dir * rotatechange
        # self.get_logger().info('set twist linear z: ' + str(twist.angular.z))
        # start rotation
        self.publisher_.publish(twist)
        # self.get_logger().info('Published twist')
        

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):       
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw_trans
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def forward(self):
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

    def stopbot(self):
        # self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
    
    def dist_check(self,path):
        '''
        if distance is larger than 1.5 return True else return False
        '''
        if len(path) == 2:
            x = path[0]
            y = path[1]
            # x = 77
            # y = 24    
            self.get_logger().info("Initializing")
            rclpy.spin_once(self)
            start = np.array((self.cur_position[0],self.cur_position[1]))
            end = np.array((x,y))        
            dist = np.linalg.norm(start-end)
            self.get_logger().info("Calculate distance")

            if (dist > 2):
                self.get_logger().info("Distance: "+ str(dist))      
                self.get_logger().info("Target at " + str(end))
                self.get_logger().info("Input position: " + str(start))
                self.get_logger().info('Rot-Yaw: R: %f D: %f' % (self.yaw_trans, np.degrees(self.yaw_trans)))
                return True
            return False
        return False
            # else:
            #     self.get_logger().info("No path found! Map complete")

    def test_move(self):
        if self.info[0] != 0 and len(self.info[1]) > 1:
            path = self.info[1].copy()
            self.get_logger().info("Adding path" + str(path))
            while (len(path)>0):
                rclpy.spin_once(self)
                point = path.pop(0)
                self.get_logger().info("Target is at: " + str(point))
                goal = Point ()
                goal.x = float(point[0])
                goal.y = float(point[1])

                inc_x = goal.x - float(self.cur_position[0])
                inc_y = goal.y - float(self.cur_position[1])

                goal_angle = (math.atan2(inc_y, inc_x) - self.yaw)

                self.rotatebot(goal_angle)
                self.get_logger().info("Finish turning")  
                time.sleep(1)
                
                while (self.dist_check(point)):
                    self.forward()
                    self.get_logger().info("Moving toward destination")
                    rclpy.spin_once(self)
                
                self.forward()
                self.get_logger().info("Moving toward destination")
                time.sleep(1)
                self.stopbot()
                self.get_logger().info("Moving finish")

    def navigation(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                self.test_move()

                # allow the callback functions to run

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
    
def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNav()
    auto_nav.navigation()
    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # rclpy.spin(auto_nav)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

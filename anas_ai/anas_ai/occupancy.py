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
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math

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
occ_bins = [-1, 0, 51, 101]
map_bg_color = 1
st = generate_binary_structure(2, 1)
dilate = 2

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/


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


def path_find(self, odata, grid_x, grid_y, height, width, explore_coordinate):

    try:

        mod_odata = np.where(np.copy(odata)-2 == 8, 0, np.copy(odata)-2)

        dilated = grey_dilation(
            mod_odata, footprint=iterate_structure(st, dilate), mode='nearest')

        matrix = np.where((dilated > 0), 0, 1)

        maximum = 0

        for coordinate in explore_coordinate:
            if (coordinate[0] < width and coordinate[1] < height):
                point1 = np.array((coordinate[0], coordinate[1]))
                point2 = np.array((grid_x, grid_y))
                dist = np.linalg.norm(point1 - point2)
                if (dist >= maximum and (dist >= 1.5) and matrix[coordinate[1], coordinate[0]] != 0):
                    maximum = dist
                    target = [coordinate[0], coordinate[1]]

        grid = Grid(matrix=matrix)
        # print("grid defined")

        start = grid.node(grid_x, grid_y)
        end = grid.node(target[0], target[1])

        # print("Node defined")

        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        path, runs = finder.find_path(start, end, grid)

        print(grid.grid_str(path=path, start=start, end=end))
        print(target)

    except Exception as e:
        print("Exception at", e)
        self.get_logger().info("Target detection error")
        return


class Occupy(Node):

    def __init__(self):
        super().__init__('occupy')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # self.publisher_ = self.create_publisher(UInt64MultiArray, 'move_path', 10)

    def listener_callback(self, msg):
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
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
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
        roll, pitch, yaw = euler_from_quaternion(
            cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

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
        self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

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
            yaw)-90, expand=True, fillcolor=map_bg_color)

        kernel = np.ones((4, 4))

        edges = np.where((odata == 3), 1, 0)
        odata_occupied = grey_dilation(
            edges, footprint=iterate_structure(st, 3), mode='nearest')

        test_img = cv2.Canny(odata, 1, 1)
        odata_unmapped = np.where(test_img > 0, 1, 0)

        lines = np.where((odata_unmapped - odata_occupied != 1), 0, 1)

        unexplored = np.where(lines == 1)
        explore_coordinate = list(zip(unexplored[1], unexplored[0]))
        # print(explore_coordinate)

        # lines = cv2.Canny(lines,1,1)

        # show the image using grayscale map
        plt.imshow(lines, cmap='gray', origin='lower')
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

        np.savetxt("map.txt", odata)

        height = msg.info.height
        width = msg.info.width

        path_find(self, odata_pass, grid_x, grid_y,
                  height, width, explore_coordinate)


def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    rclpy.spin(occupy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

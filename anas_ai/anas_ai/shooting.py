import RPi.GPIO as GPIO
import time
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16
import geometry_msgs.msg
import numpy as np

import time
import busio
import board
import adafruit_amg88xx

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(12, 1)
GPIO.output(16, 1)
GPIO.output(26, 1)
GPIO.output(21, 1)


class pewPewSick(Node):

    def __init__(self):
        # create a node called pewPewStick
        super().__init__('pewPewStick')

        # create a publishet that publishes twist data to topic 'cmd_vel'
        self.Movepublisher = self.create_publisher(
            geometry_msgs.msg.Twist, 'cmd_vel', 10)
        # create a publisher that publishes the number of heat pixels seen to topic 'pixelsSeen'
        self.pixelsSeenPublisher = self.create_publisher(
            Int16, 'pixelsSeen', 10)

        # create a subscriber that listens for the command to enter firing mode
        self.commandSubscriber = self.create_subscription(
            Int16, 'pewpewCommander', self.commandProcessor, 10)

        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.findLamp)

        # Set temperature threshold
        self.threshold = 30

        # Set firing mode to false
        self.firing = False
        self.shotCount = 0

        # timer counter
        self.timerCounter = 0

    def commandProcessor(self, msg):

        # changes to firing mode on command

        if msg.data == 1:
            self.firing = True
        elif msg.data == 0:
            self.firing = False

    def findLamp(self):
        if self.timerCounter:
            self.timerCounter -= 1
            print(
                f"ON SHOOTER DELAY, TIME LEFT = {self.timerCounter/ 4} seconds")
        else:
            logData = ""
            sumHor = 0
            sumVer = 0
            heatPixelCount = 0
            toFire = False

            # Search for heat pixels
            amgPixels = np.array(amg.pixels)
            for row in range(8):
                for col in range(8):
                    pixelData = amgPixels[row][col]
                    logData += (str(pixelData) + " ")
                    if pixelData > 30:
                        sumHor += (col - 4)
                        sumVer -= (row - 4)
                        heatPixelCount += 1
                logData += "\n"

            # if in firing mode, will take control of movement and orientate for firing
            if self.firing:
                # generates twist object
                twist = geometry_msgs.msg.Twist()

                if heatPixelCount:
                    # aims the robot if there is a heat pixel
                    aveHor = sumHor / heatPixelCount
                    aveVer = sumVer / heatPixelCount

                    if (1 < aveHor):
                        # Turn left
                        twist.angular.z = 0.9
                        logData += "Turning left \n"
                    elif (-1 > aveHor):
                        # Trun right
                        twist.angular.z = -0.9
                        logData += "Turning right \n"
                    if (1 < aveVer):
                        # pitch up
                        pitchup()
                        sleep(0.1)
                        pitchstop()
                        logData += "pitching up \n"
                    elif (-1 > aveVer):
                        # pitchdown
                        pitchdown()
                        sleep(0.1)
                        pitchstop()
                        logData += "pitching down \n"
                    if (-1 <= aveHor <= 1) and (-1 <= aveVer <= 1):
                        # Centered
                        if heatPixelCount < 4:
                            # Moves closer
                            twist.linear = 1.1
                            logData += "moving closer \n"
                        else:
                            toFire = True
                else:
                    # turns the robot around if there is no heat pixel
                    twist.angular.z = 1.1

                self.Movepublisher.publish(twist)

            if toFire:
                self.shotCount += 1
                if shotCount == 1:
                    logData += f"FIRED \n SHOTS LEFT = {6 - shotCount}"
                    # FIRE
                    fire()
                    self.timerCounter = 100
                    # First shot, longer delay (25 Seconds)
                elif shotCount < 6:
                    logData += f"FIRED \n SHOTS LEFT = {6 - shotCount}"
                    # FIRE
                    fire()
                    self.timerCounter = 52
                    # Following shots, shorter delay (13 Seconds)
                else:
                    logData += "NO SHOTS LEFT"

            heatPixelsSeen = Int16()
            heatPixelsSeen.data = heatPixelCount
            self.pixelsSeenPublisher.publish(heatPixelsSeen)

            print(logData)


def fire():
    GPIO.output(21, 0)
    sleep(0.05)
    GPIO.output(21, 1)


def pitchstop():
    GPIO.output(12, 0)
    sleep(0.05)
    GPIO.output(12, 1)


def pitchup():
    GPIO.output(16, 0)
    sleep(0.05)
    GPIO.output(16, 1)


def pitchdown():
    GPIO.output(26, 0)
    sleep(0.05)
    GPIO.output(26, 1)


def main():
    rclpy.init()

    shooter = pewPewSick()

    rclpy.spin(shooter)

    shooter.destroy_node()

    rclpy.shutdown()

    GPIO.cleanup()


if __name__ == '__main__':
    main()

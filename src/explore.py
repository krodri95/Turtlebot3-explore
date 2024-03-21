#!/usr/bin/env python3

import rospy

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the Laserscan message for subscribing to the scan topic:
from sensor_msgs.msg import LaserScan

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

# to find our explorer package path
import os
from rospkg import RosPack

# run commands in the terminal using python interface
import subprocess


class Explore():
    def odom_callback(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

        print(f"Odometry data : x={self.x}, y={self.y}, theta_z={self.theta_z}")


    def scan_callback(self, topic_data: LaserScan):
        # obtain relevant topic data:
        self.ranges = topic_data.ranges
        self.range_min = topic_data.range_min
        self.range_max = topic_data.range_max

        print(f"Laser scan data : closest object={min(self.ranges)}")


    def __init__(self):
        node_name = "explorer"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        # setup a '/cmd_vel' publisher and '/odom' and '/scan' subscribers:
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(25)  # hz

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # define the laser scan variables
        self.ranges = []
        self.range_min = 0.0
        self.range_max = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.vel_pub.publish(Twist())
        self.ctrl_c = True
        

    def main_loop(self):

        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot explorer




            # publish whatever velocity command has been set in your code above:
            self.vel_pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()


if __name__ == "__main__":
    node = Explore()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass

    # get the path of your ros package
    rp = RosPack()
    path = rp.get_path('acs6121_team23')

    # save the map of the environment
    subprocess.run(["rosrun", "map_server", "map_saver", "-f", os.path.join(path, "maps/map")])

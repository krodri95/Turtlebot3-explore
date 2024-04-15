#!/usr/bin/env python3

import rospy
from rospkg import RosPack                                  # to find our explorer package path
import subprocess                                           # run commands in the terminal using python interface

from geometry_msgs.msg import Twist                         # import the Twist message for publishing velocity commands
from nav_msgs.msg import Odometry                           # import the Odometry message for subscribing to the odom topic
from sensor_msgs.msg import LaserScan                       # import the Laserscan message for subscribing to the scan topic
from tf.transformations import euler_from_quaternion        # import the function to convert orientation from quaternions to angles

import math
from math import sqrt, pow, pi, degrees                     # import some useful mathematical operations (and pi), which you may find useful

import os
import cmath
import roslaunch
import signal

max_linear_vel = 0.2 # m/s
max_angular_vel = 1.0 # rad/s
linear_thresh = 0.35 # m
angular_thresh = 0.4 # m


class Explore():

    def lin_vel(self, d):
        fd = math.tanh(2*d/linear_thresh-2)*max_linear_vel
        return fd


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

        # assign these to class variables (so that we can
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

        #print(f"Odometry data : x={self.x}, y={self.y}, theta_z={self.theta_z}")


    def scan_callback(self, topic_data: LaserScan):
        # obtain relevant topic data:
        self.ranges = topic_data.ranges
        self.angle_min = topic_data.angle_min
        self.angle_max = topic_data.angle_max
        self.angle_increment = topic_data.angle_increment
        self.range_min = topic_data.range_min
        self.range_max = topic_data.range_max

        # consider N sections in the lidar range. Detect the closest object in each section if any.
        N = 24 #For simplicity keep this number multiple of 12
        step_angle = round(degrees(self.angle_max-self.angle_min))/N
        ob = [] # store the closest valid object
        for i in range(N):
            idx = self.ranges.index(min(self.ranges[int(i*step_angle):int((i+1)*step_angle)]))
            mod = self.ranges[idx] # modulus of complex number
            arg = idx*self.angle_increment# argument of complex number

            # Ignore distant objects i.e. larger than 40cm
            if mod <= angular_thresh:
                ob.append(cmath.rect(mod, arg))
    
        # find the weighted resultant repulsive vector
        repv = complex(0,0)
        for j in range(len(ob)):
            z_opp = cmath.rect(1/abs(ob[j])**2, cmath.phase(ob[j])+pi) # reverse the direction and change the magnitude
            wt = (1+math.cos(cmath.phase(z_opp) - pi))/2
            wt = wt if wt > 0 else 0.0
            z_opp_wt = wt*z_opp # add a weight
            repv += z_opp_wt
        
        eta = 0.25 # scaling factor
        repv = repv*eta
        dir_repv = cmath.phase(repv)

        # consider -5deg t0 +5 degree section
        indices = []
        for alpha in range(-20, 20, 1):
            if alpha < 0:
                alpha = 360+alpha
            indices.append(alpha)

        # Use a tanh function to reduce the linear velocity of the robot as it approaches an obstacle
        self.vel.linear.x = self.lin_vel(min([self.ranges[k] for k in indices]))
 
        # Calulate the magnitude and add a sign to control the direction of rotation
        mag_w = (1-math.cos(dir_repv))/2 * max_angular_vel

        mag_w = mag_w if dir_repv > pi/2 or dir_repv < -pi/2 else 0.0
        self.vel.angular.z = math.copysign(1, repv.imag) * mag_w
        #print(f"repulsive vector mag={mag_w}, direction={degrees(dir_repv)}")


    def __init__(self):
        node_name = "explorer"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        # setup a '/cmd_vel' publisher and '/odom' and '/scan' subscribers:
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        #self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)

        rospy.init_node(node_name, anonymous=True, disable_signals=True)
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
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        # slam mode start() and shutdown()
        self.slam_launch = None
        self.launch_slam_node()

        self.ctrl_c = False
        #rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

        # Register a signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)


    def signal_handler(self, sig, frame):
        rospy.loginfo("Shutting down...")
        # Call the shutdownhook method to save the map and shut down
        self.shutdownhook()


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.vel_pub.publish(Twist())
        self.ctrl_c = True

        rospy.loginfo("Saving map")
        self.save_map()

        rospy.loginfo("Shutting down the slam node")
        self.slam_launch.shutdown()

    
    def launch_slam_node(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        file_name = os.path.join(RosPack().get_path('turtlebot3_slam'), 'launch', 'turtlebot3_slam.launch')
        self.slam_launch = roslaunch.parent.ROSLaunchParent(uuid, [file_name])
        self.slam_launch.start()


    def save_map(self):
        # get the path of your ros package
        rp = RosPack()
        path = rp.get_path('acs6121_team23')

        # save the map of the environment
        subprocess.run(["rosrun", "map_server", "map_saver", "-f", os.path.join(path, "maps/map")])


    def main_loop(self):

        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot explorer

            print(f"linear velocity : {self.vel.linear.x}, angular velocity : {self.vel.angular.z}")

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

##  @ Original author: Xinyi Jiang, Jiaming Xu, Ben Winschel     ##
##  @ Oct 2017, CS department Brandeis University                ##

import rospy
import math
import time
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ScanTwistControlLoop:

    def __init__(self, scan_topic, pub_topic, policy='LHR', helper_controller=None, **kwargs):

        if policy not in ['LHR', 'RHR']:
            raise ValueError('Policy Supported: \'LHR\', \'RHR\'')
        self.direction = 1 if policy == 'LHR' else -1                   # direction 1 for left hand rule, -1 for right hand rule


        self.scan_topic_name = scan_topic
        self.pub_topic_name = pub_topic
        self.distance_to_wall = kwargs['distance_to_wall'] if 'distance_to_wall' in kwargs else 0.  # Desired distance to the wall. default 0
        self.max_speed =  kwargs['max_speed'] if 'max_speed' in kwargs else 0.                      # up-limit of robot speed. default 0


        ##/:: Register helper controller for robot movement
        if helper_controller:
            self.helper_controller = helper_controller(direction=self.direction, distance_to_wall=self.distance_to_wall)

        self.min_distance = 0.0                         # Distance closest to robot measured
        self.angle_with_closet_obstacle = 0.0           # Angle with the measured closest direction
        self.distance_front = 0.0                       # Distance at front of robot
        self.scan_sub = None                            # sensor subscriber, wait to be registered
        self.cmd_vel_pub = None                         # speed publisher, wait to be registered





    ##! Stop the robot, clear all history, reset
    def _reset_(self):
        self.twist = Twist()
        self.cmd_vel_pub(self.twist)

        self.min_distance = 0.0
        self.angle_with_closet_obstacle = 0.0
        self.distance_front = 0.0



    def start(self):

        ##/:: Register sensor data subscriber
        self.scan_sub = rospy.Subscriber(self.scan_topic_name, LaserScan, self._call_back)

        ##/:: Register robot speed publisher
        self.cmd_vel_pub = rospy.Publisher(self.pub_topic_name, Twist, queue_size=1)

        self.twist = Twist()

        while not rospy.is_shutdown():       # running until being interrupted manually
            continue
        self.cmd_vel_pub(Twist())            # Stop robot and exist




    ## Message data processing,
    ## Getting helper controller's feedback if applicable
    def _call_back(self, msg):

        ranges = [0 for i in range(360)]

        ##! Dealth with Turtlebot feature
        ##! - 360 Lidar sensor reading
        ##! - 0 reading when distance out of range
        for i in range(180):
            if msg.ranges[i + 180] == 0:
                ranges[i] = 100
            else:
                ranges[i] = msg.ranges[i + 180]

            if msg.ranges[i] == 0:
                ranges[i + 180] = 100
            else:
                ranges[i + 180] = msg.ranges[i]

        size = len(ranges)

        read_from_idx = size * (self.direction + 1) / 4        # 180-360 for direction 1 (LHR policy)
        read_to_idx = size * (self.direction + 3) / 4          # 0-180 for direction -1 (RHR policy )

        half_ranges = ranges[read_from_idx: read_to_idx]

        min_idx = ranges.index(min(half_ranges))           # Get index of direction with closest distance

        self.distance_front = ranges[size / 2]           # Get distance at front
        self.min_distance = ranges[min_idx]              # Get distance of the closest
        self.angle_with_closet_obstacle = (min_idx - size / 2) * msg.angle_increment   # Calculate angle of that closest distance


        ## Get feedback from movement helper controller
        if self.helper_controller:
            self.twist.angular.z = self.helper_controller.step(self.min_distance, self.angle_with_closet_obstacle)


        self._execute()




    ## Make decision and Execute
    def _execute(self):


        if (self.distance_front < 2 * self.distance_to_wall):
            self.twist.angular.z = self.direction * - 2.0
            print (1)

        elif (self.distance_front < self.distance_to_wall * 4):
            self.twist.linear.x = 0.5 * self.max_speed
            print (2)

        elif (math.fabs(self.angle_with_closet_obstacle) > 1.75):
            self.twist.linear.x = 0.4 * self.max_speed
            print (3)

        else:
            print (4)
            self.twist.linear.x = self.max_speed


        self.cmd_vel_pub.publish(self.twist)


        print ("Execute: ")
        print (self.twist)
        print("- - - - - - - - - -   ")
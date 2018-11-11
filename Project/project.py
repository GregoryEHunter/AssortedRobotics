from __future__ import division

import os
import json
import yaml
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos, floor
from array import *


from angles import rectify_angle_pi
from angles import degrees_to_radians

from distances import euclidian_distance


def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y,indent=4)


def yaw_from_odom(msg):
    """
    callback function to obtain yaw angle from odometry message
    """
    orientation_q = msg.pose.pose.orientation
    orientation_vec = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_vec)

    return yaw


def findObj360(array):
    """
    given an array of ranges, get the angle of the closest object around the robot
    """
    temp = min(i for i in array if i > 0.0)
    return (array.index(temp), temp)


def findObjFront(array):
    """
    given an array of ranges, get the angle of the closest object 90 degrees in front of robot 
    (45 in each direction)
    """
    temp = min(i for i in array[0:45] if i > 0.0)
    temp2 = min(i for i in array[315:360] if i > 0.0)

    if temp <= temp2:
        return (array[0:45].index(temp), temp)
    else:
        return (array[315:360].index(temp2) + 315, temp2)

class Drive:
    """
    drive the robot by a certain distance in meters forwards or backwards. If the distance is
    negative, the robot will move backwards.
    """
    def __init__(self, state, distance):
        self.state = state

        self.init_x = state.x
        self.init_y = state.y

        if distance >= 0:
            self.forward = True
        else:
            self.forward = False

        self.target_distance = abs(distance)      
        rospy.loginfo("Distance to Travel: " + str(self.target_distance))
        self.done = False

    def act(self):
        error = abs(self.target_distance - euclidian_distance(self.init_x, self.state.x, 
            self.init_y, self.state.y))
        rospy.loginfo("Current x, y: " + str( self.state.x) + str(self.state.y))

        if(error > .02):
            move_cmd = Twist()
            if self.forward:
                move_cmd.linear.x = .2
            else:
                move_cmd.linear.x = -.2
            self.state.cmd_vel.publish(move_cmd)

        else:
            self.state.cmd_vel.publish(Twist())
            self.done = True


class TurtlebotState:
    """
    stores the current state of the robot.
    """
    def __init__(self):

        self.pose_msg = None
        self.yaw_msg = None
        self.scan_msg = None

        self.dict = {"position":None, "orientation" : None, "scan" : None}

        # start up the subscribers to monitor state
        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.update_scan)


        # remove the robot data file
        self.filename = "robot.txt"
        if os.path.exists(self.filename):
            os.remove(self.filename)

        self.write_num = 0
        #self.data_to_file()

        self.angle = None
        self.x = None
        self.y = None
        self.ready = False
        self.current_action = None
        self.meter = False

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        # wait until sensing received, etc before
        # returning control
        while not self.ready:
            rate = rospy.Rate(20)
            rate.sleep()


    def update_odom(self, msg):
        """
        updates odometry information of the robot.
        """
        self.pose_msg = msg.pose.pose.position

        self.angle = yaw_from_odom(msg)
        self.yaw_msg = self.angle

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.ready = True


    def update_scan(self, msg):
        """
        updates laser range finder environment information around the robot.
        """
        self.scan_msg = msg.ranges

        # in 360 degree range
        self.closest_obj_ang = degrees_to_radians(findObj360(msg.ranges)[0])

        # in 90 degree range in front
        self.closest_obj_front = findObjFront(msg.ranges)
        self.closest_obj_front_ang = degrees_to_radians(self.closest_obj_front[0])
        self.closest_obj_front_dist = self.closest_obj_front[1]
        #self.Hough_T = Hough_Transform(msg.ranges)

        self.ready = True


    def data_to_file(self):

        # convert data to JSON format
        rospy.Rate(5).sleep()

        self.dict["position"] = msg2json(self.pose_msg)
        rospy.Rate(5).sleep()

        # string manipultation and dictionary addition for scanning data
        self.dict["scan"] = "{\"ranges\" : " + str(self.scan_msg) + "}"
        rospy.Rate(5).sleep()

        # string manipultation and dictionary addition for yaw data
        self.dict["orientation"] = "{\"yaw\" : " + str(self.yaw_msg) + "}"

        # append each new json blob to a new line
        with open(self.filename, "a") as file:
            file.write("{\""+str(self.write_num)+ "\": {")
            for key, value in self.dict.iteritems():
                if value:
                    val = value
                    if (key != "position"):
                        val = str(val).replace("\n", ",")
                    else:
                        val = str(val).replace("\n", " ")

                    # all values should be comma separated, lists should have square brackets
                    val = str(val).replace("(", "[")
                    val = str(val).replace(")", "]")

                    file.write("\""+str(key) + "\"" + ":" + str(val))
                if(key != "scan"):
                    file.write(",")
            file.write("}}\n")
        self.write_num += 1


    def shutdown(self):
        """
        shutsdown the robot.
        """
        rospy.loginfo("Shutting down turtlebot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye.")



def main():
    rospy.init_node("turn_to")
    state = TurtlebotState()
    rospy.on_shutdown(state.shutdown)
    rate = rospy.Rate(20)

    # pause for a bit
    # for i in range(50):
    #    rate.sleep()


    # distance = 2
    # state.current_action = Drive(state, distance)

    while not rospy.is_shutdown():
        state.data_to_file()

        # print("here")
        # if not state.current_action.done:
        # state.data_to_file
            # state.current_action.act()
        # else:
        #     break
        rate.sleep()


main()






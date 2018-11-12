from __future__ import division

import rospy
import numpy as np

import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos, floor
from array import *


from angles import rectify_angle_pi
from angles import degrees_to_radians

from distances import euclidian_distance


def yaw_from_odom(msg):
    """
    callback function to obtain yaw angle from odometry message
    """
    orientation_q = msg.pose.pose.orientation
    orientation_vec = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_vec)

    return yaw


def findLandmarks(robot_x, robot_y, robot_head, array, n = 3):
    """
    given an array of ranges, get n closest landmarks around the robot

    The landmark list will consist of a list of x,y coordinates computed as follows:
        landmark_x = robot_x + radius * cos(theta + robot_heading)
        landmark_y = robot_y + radius * sin(theta + robot_heading)
    """

    # get the indicies of n minimum values
    arr = np.array(array)
    landmark_indicies = arr.argsort()[:n]

    """
    for each index returned, store a tuple containing the n closest objects 
    theta = index
    r = value
    """
    landmark_list = []
    for index in landmark_indicies:
        landmark_list.append([robot_x + array[index] * cos(degrees_to_radians(index) + robot_head), 
            robot_y + array[index] * sin(degrees_to_radians(index) + robot_head)])

    print(landmark_list)
    return landmark_list


class TurtlebotState:
    """
    stores the current state of the robot.
    """
    def __init__(self):

        # start up the subscribers to monitor state
        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.update_scan)


        """
        save message contents for filing
        """
        self.pose_msg = None
        self.yaw_msg = None
        self.scan_msg = None

        self.dict = {"position":None, "orientation" : None, "scan" : None}

        """
        robot save data to file
        """
        # remove the robot data file
        # self.filename = "robot.txt"
        # if os.path.exists(self.filename):
        #     os.remove(self.filename)

        # self.write_num = 0
        #self.data_to_file()

        """
        keep running total of global position based on odom data
        """
        self.angle = None
        self.x = None
        self.y = None

        """
        keep running total of error terms
        """
        self.x_err = 0
        self.y_err = 0
        self.angle_err = 0

        """
        store previous and current landmark list for comparisson purposes
        """
        self.prev_landmarks = None
        self.curr_landmarks = None


        self.ready = False
        self.current_action = None


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

        self.ready = True

    def set_prev_landmarks(self):
        if self.prev_landmarks is None:
            self.prev_landmarks = findLandmarks(self.x, self.y, self.angle, self.scan_msg)


    def update_error(self):

        print("updating the error terms")
        # get current landmark data 
        # NOTE: (not sure if this should be based on self.x or self.x + self.x_err)
        self.curr_landmarks = findLandmarks(self.x, self.y, self.angle, self.scan_msg)
        rospy.sleep(1)


        # calculate where we expect the prev_landmarks to be with the change in odom data
        # CODE GOES HERE

        # find difference (error) in x,y,theta between current landmarks and expectations
        # CODE GOES HERE

        # update x_err, y_err, angle_err
        # CODE GOES HERE



        # let prev_landmarks = curr_landmarks
        self.prev_landmarks = self.curr_landmarks




        self.ready = True

    # def data_to_file(self):
    #     # convert data to JSON format
    #     rospy.Rate(5).sleep()
    #     # string manipulation and dictionary addition for position
    #     self.pose_msg = "\"" + str(self.pose_msg).replace("\n", "\n\"")
    #     self.pose_msg = str(self.pose_msg).replace(":", "\":")
    #     self.dict["position"] = str(self.pose_msg)
    #     rospy.Rate(5).sleep()

    #     # string manipultation and dictionary addition for scanning data
    #     self.dict["scan"] = "\"ranges\" : " + str(self.scan_msg)
    #     rospy.Rate(5).sleep()

    #     # string manipultation and dictionary addition for yaw data
    #     self.dict["orientation"] = "\"yaw\" : " + str(self.yaw_msg)

    #     with open("robot.txt", "w") as file:
    #         file.write("{\"robot\": {\n")
    #         for key, value in self.dict.iteritems():
    #             if value:
    #                 # all values should be comma separated, lists should have square brackets
    #                 val = str(value).replace("\n", ",\n")
    #                 val = str(val).replace("(", "[")
    #                 val = str(val).replace(")", "]")

    #                 file.write("\""+str(key) + "\"" + ":{ \n" + str(val) + "}")
    #             if(key != "scan"):
    #                 file.write(",\n")
    #         file.write("}}")


    def shutdown(self):
        """
        shutsdown the robot.
        """
        rospy.loginfo("Shutting down turtlebot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye.")


    # def stateEstimate(self):
    #   threading.Timer(2.0, printit).start()
    #   print "Hello, World!"




def main():
    rospy.init_node("turn_to")
    state = TurtlebotState()
    rospy.on_shutdown(state.shutdown)
    rate = rospy.Rate(20)

    # pause for a bitr
    for i in range(10):
       rate.sleep()

    state.set_prev_landmarks()
    # distance = 2
    # state.current_action = Drive(state, distance)

    count = 0
    while not rospy.is_shutdown():
        # less elegent way to run a sequence of functions approx every 2 seconds
        if count == 2:
            state.update_error()

            count = 0
        count += 1

        # print("here")
        # if not state.current_action.done:
        # state.data_to_file
            # state.current_action.act()
        # else:
        #     break
        rate.sleep()

main()

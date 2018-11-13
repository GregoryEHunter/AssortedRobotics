from __future__ import division

import rospy
import numpy as np

import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import *
from array import *


from angles import rectify_angle_pi
from angles import rectify_angle_2pi
from angles import degrees_to_radians

from distances import euclidian_distance


global counter 
global Points_in_boxes
Points_in_boxes = [[ [] for i in range(45)] for j in range(150)]
counter = 0

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


def Build_XY(array_x, array_y):
    #xyArray = [[0 for col in range(2)] for row in range(len(array_x))]
    xyArray = []
    #print("Length of array_x %d", len(array_x))
    
    for x_cord in array_x:
        theta = atan2(array_y[array_x.index(x_cord)],x_cord)
        x = x_cord
        y = array_y[array_x.index(x_cord)]
        xyArray.append((theta, x, y))
        #print("%d,%d,%d",xyArray[0][array_x.index(x_cord)],xyArray[1][array_x.index(x_cord)],xyArray[2][array_x.index(x_cord)])
        #print(xyArray)
        #print("looping")

    return xyArray


def Hough_Transform(xyArray):
    global Points_in_boxes
    print("Entered Hough")

    boxes_theta = 45    # num of boxes of angles
    boxes_rho = 100   # num of boxes for rho
    max_rho = 8         # min rho is 0
    thresh = 750

    # precomputations to save time
    ratio_boxPerRho = boxes_rho/max_rho
    ratio_boxPerTheta = boxes_theta/360

    ratio_thetaPerBox = 360/boxes_theta
    ratio_rhoPerBox = max_rho/boxes_rho

    
    # initialize xy array with 2 cols and 360 rows
    #xyArray = [[0 for col in range(2)] for row in range(360)]

    #for theta in range(len(array)):
        #xyArray.insert(theta, [array[theta]*cos(np.deg2rad(theta)), array[theta]*sin(np.deg2rad(theta))])
        #xyArray.insert(theta, [array[theta]*cos(np.deg2rad(theta)), array[theta]*sin(np.deg2rad(theta))])


    # accumulator based on rho and theta with all values initialized to 0's
    accumulator = np.zeros((boxes_rho, boxes_theta))

    

    #print("Rho: {}, X: {}, Y: {}").format(xyArray[0][0],xyArray[1][0],xyArray[2][0])


    # for each x,y point
    for xy in range(len(xyArray)):
        # for each theta angle
        for theta in range(360):
            # calculate rho
            rho = xyArray[xy][1] * cos(np.deg2rad(theta)) + xyArray[xy][2] * sin(np.deg2rad(theta))
            
            # not inclusive of max_rho
            if (rho < max_rho) and (rho > 0):
                box = (int(ratio_boxPerRho * rho), int(ratio_boxPerTheta * theta))

                Points_in_boxes[int(ratio_boxPerRho * rho)][int(ratio_boxPerTheta * theta)].append((xyArray[xy][1],xyArray[xy][2]))


                # increment the respective rho, theta box
                accumulator[box[0], box[1]]+=1




    # figure out all points in accumulator that are greater than thresh
    indicies = np.argwhere(accumulator > thresh)       # returns all indicies in array where boolean condition matches

    #coordinate = [[0 for col in range(len(indicies))] for row in range(2)]
    coordinate = []



    for index_pair in range(len(indicies)):
        rho_box = indicies[index_pair][0]
        theta_box = indicies[index_pair][1]

        # get midpoints of each box
        coordinate.append((rho_box * ratio_rhoPerBox + ratio_rhoPerBox/2, 
            theta_box * ratio_thetaPerBox + ratio_thetaPerBox/2))


    closest_dist = max_rho

    for i in range(len(coordinate)):
        curr = coordinate[i][0]
        print("Theta=%d, Rho=%d", coordinate[i][1],coordinate[i][0])


        #if curr < closest_dist:
            #closest_dist = curr
            #goal_angle = coordinate[i][1]      # get goal angle

    #return goal_angle



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
        self.x_prev = None
        self.y_prev = None

        self.angle_init = None
        self.all_data = []
        self.all_data_x = []
        self.all_data_y = []
        self.data_taken_x = []
        self.data_taken_y =[]


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

        if self.ready == False:
            self.x_prev = self.x
            self.y_prev = self.y
            self.angle_init = self.angle

        self.ready = True



    def update_scan(self, msg):
        global counter
        """
        updates laser range finder environment information around the robot.
        """
        self.scan_msg = msg.ranges
        #if  counter%20 == 0:
        if euclidian_distance(self.x,self.x_prev,self.y,self.y_prev) > 1:
            for theta in range(360):
                if self.scan_msg[theta] > .05 and self.scan_msg[theta] < 4:
                    #self.all_data.append([self.scan_msg[theta]*cos(np.deg2rad(theta)) + self.x, self.scan_msg[theta]*sin(np.deg2rad(theta)) + self.y])
                    #x = [self.scan_msg[theta]*cos(np.deg2rad(theta)) + self.x]
                    #y = [self.scan_msg[theta]*sin(np.deg2rad(theta)) + self.y]
                    self.all_data_x.append(self.scan_msg[theta]*cos(np.deg2rad(theta)+rectify_angle_2pi(self.angle)) + self.x)
                    self.all_data_y.append(self.scan_msg[theta]*sin(np.deg2rad(theta)+rectify_angle_2pi(self.angle)) + self.y)
            self.data_taken_x.append(self.x)
            self.data_taken_y.append(self.y)
            self.x_prev = self.x
            self.y_prev = self.y
            print("Updated map!")
        counter += 1


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
    global counter
    global Points_in_boxes
    rospy.init_node("turn_to")
    state = TurtlebotState()
    rospy.on_shutdown(state.shutdown)
    rate = rospy.Rate(20)

    # pause for a bitr
    for i in range(10):
       rate.sleep()


    while not rospy.is_shutdown():
        #if counter%60 == 0:
        rate.sleep()
            

    print(state.all_data_x)
    print(" ---------------  ")
    print("\n")
    print(state.all_data_y)
    print(" ***********************  ")
    print("\n\n\n\n\n")
    print(state.data_taken_x)
    print(" ~~~~~~~~~~~~~~~  ")
    print(state.data_taken_y)


    print("%%%%%%%%%%%%%%%%%%\n" + str(Points_in_boxes) + "%%%%%%%%%%%%%%%\n\n\n")


    
    Hough_Transform(Build_XY(state.all_data_x, state.all_data_y))

    # state.set_prev_landmarks()
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

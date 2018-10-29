import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi
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

def Hough_Transform(array):

    precision = 50
    dist_min = .05
    dist_max = 4
    threshold = 5

    #Take all the ranges (and their corresponding angle) and convert to x,y coordinates for each point
    #make an array of 360 rows and 2 columns where each row is a point
    Cartesian = [[0 for col in range(2)] for row in range(360)]
    for i in range(360):
        Cartesian.insert(i, [array[i]*np.cos(np.deg2rad(i)),array[i]*np.sin(np.deg2rad(i))])

    #only look at lines that are betwen 5cm and 4m away at their closest point
    theta = np.linspace(0,2*pi, precision)
    r = np.linspace(dist_min,dist_max, precision)

    #2D array which is #of discreet thetas by # of discrete distances
    lines = [0 for col in range(theta) for row in range(r)]

    for i in range(360):
        r  = Cartesian[i][1]*np.cos(np.deg2rad(i)) + Cartesian[i][2]*np.sin(np.deg2rad(i))

        if r > dist_min & r < dist_max:
            #integer division to put r/theta into a discrete box on the hough map
            r = (r-dist_min)//(len(r))
            t = np.deg2rad(i)//len(theta)
            lines[r][t] += 1

    #If any of the squares in the map are above the threshold, they represent a line the robot detected
    for i in range(len(r)):
        for j in range(len(t))
            if lines[i][j] > threshold:
                rospy.loginfo("Line with coordinates ({},{}) in polar").format(i,j)






class Turn:
    """
    turn the robot by an angle defined in radians, if angle is defined as positive the robot will 
    turn clockwise
    """
    def __init__(self, state, angle):
        self.state = state

        if angle >= 0:
            self.clockwise = False
        else:
            self.clockwise = True
        
        self.target_angle = rectify_angle_pi(state.angle + angle)
        
        rospy.loginfo("Target angle: " + str( self.target_angle))
        self.done = False

    def act(self):
        error = abs(self.target_angle - self.state.angle)
        rospy.loginfo("Current angle: " + str( self.state.angle))

        if(error > .02):
            move_cmd = Twist()
            if self.clockwise:
                move_cmd.angular.z = -.2
            else:
                move_cmd.angular.z = .2
            self.state.cmd_vel.publish(move_cmd)

        else:
            self.state.cmd_vel.publish(Twist())
            self.done = True


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


class TurnToObject:
    """
    the robot uses tha range finder to grab the direction of the closest object and rotate such 
    that its heading angle points towards the closest object.
    """
    def __init__(self, state):
        self.state = state
        self.done = False

    def act(self):
        goal = rectify_angle_pi(self.state.closest_obj_ang)
        rospy.loginfo("Angle of Closest Object: " + str(goal))
        rospy.loginfo("Angle of Closest Object in Front: " + str(rectify_angle_pi(
            self.state.closest_obj_front_ang)))

        self.state.current_action = Turn(self.state, goal)
        self.done = True


class FollowObject:
    """
    the robot follows the closest object in front of it at an angle of +pi/2 and -pi/2 by sending
    angular and linear command velocities.
    """
    def __init__(self, state):
        self.state = state

        # error and bound limit constants
        self.lower_bound = 0.4
        self.upper_bound = 0.6
        self.angle_err_lim = 0.04

        goal_angle = rectify_angle_pi(self.state.closest_obj_front_ang)

        if goal_angle >= 0:
            self.clockwise = False
        else:
            self.clockwise = True
        
        self.target_angle = rectify_angle_pi(self.state.angle + goal_angle)
        
        self.done = False

    def act(self):
        goal = rectify_angle_pi(self.state.closest_obj_front_ang)
        error = abs(self.target_angle - self.state.angle)

        move_cmd = Twist()

        # sends an angular command velocity if the current robot angle is off by .04 radians
        if(error > self.angle_err_lim):
            if self.clockwise:
                move_cmd.angular.z = -.4
            else:
                move_cmd.angular.z = .4
        else:
            move_cmd.angular.z = 0
        
        # sends a linear command velocity if the closest object distance is not between 0.4 and 0.6m
        if (self.state.closest_obj_front_dist > self.upper_bound or 
            self.state.closest_obj_front_dist < self.lower_bound):
            if(self.state.closest_obj_front_dist - 0.5 > 0):
                move_cmd.linear.x = .15
            else:
                move_cmd.linear.x = -.15

        else:
            move_cmd.linear.x = 0

        self.state.cmd_vel.publish(move_cmd)

        # sends 0 command velocity if the robot is in a desired position
        if(error <= self.angle_err_lim and (self.state.closest_obj_front_dist <= self.upper_bound 
                and self.state.closest_obj_front_dist >= self.lower_bound)):
            move_cmd = Twist()
            self.state.cmd_vel.publish(Twist())
        
        self.done = True              


class WallFollow:
    def __init__(self, state):
        self.state = state

        # error and bound limit constants
        self.lower_bound = 0.9
        self.upper_bound = 1.1
        self.angle_err_lim = 0.04


        goal_angle = rectify_angle_pi(self.state.closest_obj_front_ang)

        if goal_angle >= 0:
            self.clockwise = False
        else:
            self.clockwise = True
        
        self.target_angle = rectify_angle_pi(self.state.angle + goal_angle)
        
        self.done = False

    def act(self):
        goal = rectify_angle_pi(self.state.closest_obj_front_ang)
        error = abs(self.target_angle - self.state.angle)

        move_cmd = Twist()

        if(error > self.angle_err_lim):
            if self.clockwise:
                move_cmd.angular.z = -.4
            else:
                move_cmd.angular.z = .4
        else:
            move_cmd.angular.z = 0
        
        if (self.state.closest_obj_front_dist > self.upper_bound or self.state.closest_obj_front_dist < self.lower_bound):
            if(self.state.closest_obj_front_dist - 0.5 > 0):
                move_cmd.linear.x = .15
            else:
                move_cmd.linear.x = -.15

        else:
            move_cmd.linear.x = 0

        self.state.cmd_vel.publish(move_cmd)
        print(str(error) + "\tERROR")
        print(str(self.state.closest_obj_front_dist) + "closest_obj_front_dist")
        if(error <= self.angle_err_lim and (self.state.closest_obj_front_dist <= self.upper_bound and self.state.closest_obj_front_dist >= self.lower_bound)):
            move_cmd = Twist()
            print("setting meter to true GOTHERE")
            self.state.cmd_vel.publish(Twist())
            self.state.meter = True
        
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

        self.data_to_file()

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

        self.ready = True

    def data_to_file(self):

        # convert data to JSON format
        rospy.Rate(5).sleep()
        # string manipulation and dictionary addition for position
        self.pose_msg = "\"" + str(self.pose_msg).replace("\n", "\n\"")
        self.pose_msg = str(self.pose_msg).replace(":", "\":")
        self.dict["position"] = str(self.pose_msg)
        rospy.Rate(5).sleep()

        # string manipultation and dictionary addition for scanning data
        self.dict["scan"] = "\"ranges\" : " + str(self.scan_msg)
        rospy.Rate(5).sleep()

        # string manipultation and dictionary addition for yaw data
        self.dict["orientation"] = "\"yaw\" : " + str(self.yaw_msg)

        with open("robot.txt", "w") as file:
            file.write("{\"robot\": {\n")
            for key, value in self.dict.iteritems():
                if value:
                    # all values should be comma separated, lists should have square brackets
                    val = str(value).replace("\n", ",\n")
                    val = str(val).replace("(", "[")
                    val = str(val).replace(")", "]")

                    file.write("\""+str(key) + "\"" + ":{ \n" + str(val) + "}")
                if(key != "scan"):
                    file.write(",\n")
            file.write("}}")

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
    for i in range(20):
        rate.sleep()

    # #####################################
    # #Follow Object Code
    # # turn to closest object
    # state.current_action = TurnToObject(state)
    # while not rospy.is_shutdown():
    #     if not state.current_action.done:
    #         state.current_action.act()
    #     else:
    #         break
    #     rate.sleep()

    # state.current_action = FollowObject(state)
    # while not rospy.is_shutdown():
    #     if not state.current_action.done:
    #         state.current_action.act()
    #     else:
    #          state.current_action = WallFollow(state)
    #     rate.sleep()
    # #####################################


 #    ######################################
 #    #Wall Follow Code:
    # # turn to closest object
 #    state.current_action = TurnToObject(state)
 #    while not rospy.is_shutdown():
 #        if not state.current_action.done:
 #            state.current_action.act()
 #        else:
 #            break
 #        rate.sleep()

 #    state.current_action = WallFollow(state)
 #    while not rospy.is_shutdown():
 #        rospy.loginfo("state.meter" + str(state.meter))
 #        if not state.current_action.done:
 #            state.current_action.act()
 #        elif state.meter is True:
 #            break
 #        else:
 #             state.current_action = WallFollow(state)
 #        rate.sleep()

 #    state.current_action = Turn(state,-pi/2)
 #    while not rospy.is_shutdown():
 #        if not state.current_action.done:
 #            state.current_action.act()
 #        else:
 #            break
 #        rate.sleep()

 #    state.current_action = Drive(state, 1)
 #    while not rospy.is_shutdown():
 #        if not state.current_action.done:
 #            state.current_action.act()
 #        else:
 #            break
 #        rate.sleep()
 #    #########################################


main()



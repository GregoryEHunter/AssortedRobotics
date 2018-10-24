import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi

from angles import rectify_angle_pi
from angles import degrees_to_radians

from distances import euclidian_distance

def yaw_from_odom(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_vec = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_vec)

    return yaw

# given an array of ranges, get the angle of the closest object around the robot
def findObj360(array):
    temp = min(i for i in array if i > 0.0)
    return (array.index(temp), temp)

# given an array of ranges, get the angle of the closest object 90 degrees in front of robot (45 in each direction)
def findObjFront(array):
    temp = min(i for i in array[0:45] if i > 0.0)
    #print(array[0:45].index(temp))
    temp2 = min(i for i in array[315:360] if i > 0.0)
    #print(array[315:360].index(temp2) +315)

    if temp <= temp2:
        return (array[0:45].index(temp), temp)
    else:
        return (array[315:360].index(temp2) + 315, temp2)

def findObjSide(array):
    dist = min(i for i in array[60:120] if i > 0.0)
    return (dist)

class Turn:
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

# drive a certain distance forward
class Drive:
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
        error = abs(self.target_distance - euclidian_distance(self.init_x, self.state.x, self.init_y, self.state.y))
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

#Looks to the right of the robot (over a 60 degree arc, 30 on each side of 90)
#Takes the closest object and turns left if too close, right if too far
#keeps track of how far off the robots original heading so as not to continue turning
#(Or to only turn once if not 1 meter away fromt the wall)
class FollowWall:
    def __init__(self, state):
        self.state = state
        distance = self.state.closest_obj_side

        self.done = False
    

    def act(self):
        move_cmd = Twist()

        if (distance > 1):
            if self.FakeHeading >= 0:
                move_cmd.angular.z = -.4
                self.FakeHeading -= 1
        elif (distance < 1):
            if self.FakeHeading <=0:
                move_cmd.angular.z = .4
                self.FakeHeading += 1
        else:
            move_cmd.angular = 0

        
        move_cmd.linear.x = .15

        self.state.cmd_vel.publish(move_cmd)


        self.done = True



# scan for closest object around it and turn towards it
class TurnToObject:
    def __init__(self, state):
        self.state = state
        self.done = False

    def act(self):
        # turn to the angle closest to the object
        goal = rectify_angle_pi(self.state.closest_obj_ang)
        rospy.loginfo("Angle of Closest Object: " + str(goal))
        rospy.loginfo("Angle of Closest Object in Front: " + str(rectify_angle_pi(self.state.closest_obj_front_ang)))

        self.state.current_action = Turn(self.state, goal)
        self.done = True


class FollowObject:
    def __init__(self, state):
        self.state = state
        goal_angle = rectify_angle_pi(self.state.closest_obj_front_ang)

        if goal_angle >= 0:
            self.clockwise = False
        else:
            self.clockwise = True
        
        self.target_angle = rectify_angle_pi(self.state.angle + goal_angle)
        
        self.done = False

    def act(self):
        # follow closest object that is in the -pi/4 to pi/4 range
        goal = rectify_angle_pi(self.state.closest_obj_front_ang)
        error = abs(self.target_angle - self.state.angle)

        move_cmd = Twist()

        if(error > .04):
            if self.clockwise:
                move_cmd.angular.z = -.4
            else:
                move_cmd.angular.z = .4
        else:
            move_cmd.angular.z = 0
        
        # self.state.cmd_vel.publish(move_cmd)

        # move_cmd = Twist()
        if (self.state.closest_obj_front_dist > .6 or self.state.closest_obj_front_dist < .4):
            if(self.state.closest_obj_front_dist - 0.5 > 0):
                move_cmd.linear.x = .15
            else:
                move_cmd.linear.x = -.15

        else:
            move_cmd.linear.x = 0

        self.state.cmd_vel.publish(move_cmd)

        if(error <= .04 and (self.state.closest_obj_front_dist <= .6 and self.state.closest_obj_front_dist >= .4)):
            move_cmd = Twist()
            self.state.cmd_vel.publish(Twist())
        
        self.done = True              


class TurtlebotState:
    def __init__(self):
        # start up the subscribers to monitor state

        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.subscriber_scan = rospy.Subscriber("/scan", LaserScan, self.update_scan)

        self.angle = None
        self.x = None
        self.y = None
        self.ready = False
        self.current_action = None
        self.FakeHeading = 0

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        # wait until sensing received, etc before
        # returning control
        while not self.ready:
            rate = rospy.Rate(20)
            rate.sleep()

    def update_odom(self, msg):
        self.angle = yaw_from_odom(msg)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.ready = True

    def update_scan(self, msg):
        # rospy.loginfo(msg)
        # rospy.loginfo(len(msg.ranges))
        # rospy.loginfo(msg.range_min)
        # rospy.loginfo(min(msg.ranges))
        # rospy.loginfo(msg.ranges.index(min(msg.ranges)))

        # in 360 degree range
        self.closest_obj_ang = degrees_to_radians(findObj360(msg.ranges)[0])

        # in 90 degree range in front
        self.closest_obj_front = findObjFront(msg.ranges)
        self.closest_obj_front_ang = degrees_to_radians(self.closest_obj_front[0])
        self.closest_obj_front_dist = self.closest_obj_front[1]


        #Looking to the right side over a 60 degree arc
        self.closest_obj_side = findObjSide(msg.ranges)

        self.ready = True

    def shutdown(self):

        rospy.loginfo("Shutting down turtlebot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye.")



# TO DO (for prob 1)
# how to queuee commands?

def main():
    rospy.init_node("turn_to")

    state = TurtlebotState()

    rospy.on_shutdown(state.shutdown)

    rate = rospy.Rate(20)


    # pause for a bit
    for i in range(20):
        rate.sleep()


    # turn to another angle (away from an object)
    # state.current_action = Turn(state, -pi/2)

    # while not rospy.is_shutdown():
    #     if not state.current_action.done:
    #         state.current_action.act()
    #     else:
    #         break
    #     rate.sleep()


    # turn to closest object
    state.current_action = TurnToObject(state)

    while not rospy.is_shutdown():
        if not state.current_action.done:
            state.current_action.act()
        else:
            break

        rate.sleep()

    state.current_action = FollowObject(state)
    while not rospy.is_shutdown():
        if not state.current_action.done:
            state.current_action.act()
        else:
            state.current_action = FollowObject(state)

        rate.sleep()


    # drive the robot forwards or backwards
    # state.current_action = Drive(state, -.5)
    # while not rospy.is_shutdown():
    #     if not state.current_action.done:
    #         state.current_action.act()
    #     else:
    #         break

    #     rate.sleep()


    #Follow the Wall

    # turn to closest object
    state.current_action = FollowWall(state)

    while not rospy.is_shutdown():
        if not state.current_action.done:
            state.current_action.act()
        else:
            state.current_action = FollowWall(state)

        rate.sleep()



main()



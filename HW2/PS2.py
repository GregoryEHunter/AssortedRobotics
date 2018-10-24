#The main structure of this code is taken from the quickstart guide
#provided by Prof. Balkom

import rospy
from geometry_msgs.msg import Twist

#4a.
def timed_drive(sec):
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(1) #rate of 1Hz

    move_cmd = Twist()
    move_cmd.linear.x = .2 #Move at .2 m/s

    for i in range(sec+1): #time input for length of drive
        cmd_vel.publish(move_cmd)
        r.sleep() #sleep for 1 sec


#4b.
def timed_right(sec):
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(1) #rate of 1Hz

    move_cmd = Twist()
    move_cmd.angular.z = -1 #turn at .2 r/s

    for i in range(sec+1): #time input for length of drive
        cmd_vel.publish(move_cmd)
        r.sleep() #sleep for 1 sec


#4b.
def timed_left(sec):
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(1) #rate of 1Hz

    move_cmd = Twist()
    move_cmd.angular.z = 1 #Move at .2 m/s

    for i in range(sec+1): #time input for length of drive
        cmd_vel.publish(move_cmd)
        r.sleep() #sleep for 1 sec

def init():
    rospy.init_node('GoForward', anonymous=False)

def main():

    timed_drive(3)
    timed_left(2)
    timed_right(2)

def stop():
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Stop turtlebot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)



init()
main()
stop()
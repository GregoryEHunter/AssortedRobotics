# Methods to Control turtlebots. Will turn the turtlebot,
# move an arbitrary distance, and will move the turtlebot
# so as to move a pen fixed at a distance in front of the
# turtlebot at an arbitrary rate.

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import pi
import math
from tf.transformations import euler_from_quaternion


#Translates from Quaternion odometry angles to euler
def yaw_from_odom(msg):
	orientation_q = msg.pose.pose.orientation
	orientation_vec = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_vec)
	return yaw

class RobotState:
	def __init__(self):
		self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
		self.angle = None
		self.x = None
		self.y = None

	def update_odom(self, msg):
		self.angle = yaw_from_odom(msg)
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

	def get_curr_angle(self):
		return self.angle

	def get_curr_y(self):
		return self.y

	def get_curr_x(self):
		return self.x


#Drives a given distance (in meters) by checking linear distance
#from an initial point. Uses odemtry to measure distance travelled
#in x & y
def Drive(distance):

	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	r = rospy.Rate(50) #update at 1/50 sec
	move_cmd = Twist()
	reverse = False
	init_x = mybot.get_curr_x() #initial x and y positions
	init_y = mybot.get_curr_y()

	#Check to check direction of travel and set speed
	if distance < 0:
		reverse = True
		distance = abs(distance)
	if reverse:
		move_cmd.linear.x = -.2
	else:
		move_cmd.linear.x = .2

	#while not within .2m of desired distance, drive	
	while math.sqrt(math.pow(init_x-mybot.get_curr_x(),2)+math.pow(init_y-mybot.get_curr_y(),2)) < (distance-.2):
		cmd_vel.publish(move_cmd)
		r.sleep()

	#momentary stop
	move_cmd.linear.x = 0
	cmd_vel.publish(move_cmd)

	#Drive slower within .2 meters
	if reverse:
		move_cmd.linear.x = -.1
	else:
		move_cmd.linear.x = .1
	while math.sqrt( math.pow(init_x-mybot.get_curr_x(),2)+math.pow(init_y-mybot.get_curr_y(),2)) < distance:
		cmd_vel.publish(move_cmd)
		r.sleep()

	#stop
	move_cmd.linear.x = 0
	cmd_vel.publish(move_cmd)



#Turns to desired Angle on the unit circle - takes desired
#angle in degrees. Uses odometry to check current angle and 
#continues to current angle until within 1.5 degrees. Also
#calculates which direction so as to turn to turn shorter 
#distance
def TurnTo(to_Angle):

	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	r = rospy.Rate(50) #update at 1/50 sec
	move_cmd = Twist()
	Clockwise = True

	#Calculate which direction to turn
	if ((to_Angle-(mybot.get_curr_angle()*360/(2*pi)+180)+540)%360-180 <= 0):
		move_cmd.angular.z = -pi/2 #clockwise from above
	else:
		move_cmd.angular.z = pi/2  #counterclockwise from above
		Clockwise = False

	print"Desination: {}, CurrAngle: {}, Clockwise: {}".format(to_Angle, (mybot.get_curr_angle()*360/(2*pi)+180), Clockwise)

	#Move quickly until within 20 degrees of desired
	while(abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) > 20 and abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) < 340 ):
		cmd_vel.publish(move_cmd)
		r.sleep()

	#active momentum reduction
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)

	move_cmd.angular.z = pi/16
	if Clockwise:
		move_cmd.angular.z = -pi/16

	#Move slowly until within 1.5 degrees - results in within .5 degrees on average
	while(abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) > 1.5 and abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) < 358.5):
		cmd_vel.publish(move_cmd)
		r.sleep()

	#stop
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)

#Turns the given amount by looking at odometry and continuing to turn
#until within 1.5 degrees. Turns the direction given by the sign
#of the input. Takes the argument in radians and immmeadiately converts
#to degrees
def Turn(degrees):

	degrees = degrees*360/(2*pi)
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	r = rospy.Rate(50) #update at 1/50 sec
	move_cmd = Twist()
	
	#Decide which direction to turn
	Clockwise = True
	if degrees < 0:
		move_cmd.angular.z = -pi/2
	else:
		move_cmd.angular.z = pi/2
		Clockwise = False

	#Decide where to turn to - take into account overflows and turns greater than 2pi
	if degrees > 360:
		degrees = degrees % (360)
	while degrees <-360:
		degrees = degrees+360
	if ((mybot.get_curr_angle()*360/(2*pi)+180) + degrees) < -360:
		to_Angle = (mybot.get_curr_angle()*360/(2*pi)+180) + degrees + 360
	elif ((mybot.get_curr_angle()*360/(2*pi)+180) + degrees) > 360:
		to_Angle = (mybot.get_curr_angle()*360/(2*pi)+180) + degrees - 360
	else:
		to_Angle = degrees + (mybot.get_curr_angle()*360/(2*pi)+180)
		

	print "Going to {} from {} which is a {} movement".format(to_Angle, (mybot.get_curr_angle()*360/(2*pi)+180), degrees)

	#Move quickly until within 20 degrees of desired
	while(abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) > 20 and abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) < 340 ):
		cmd_vel.publish(move_cmd)
		r.sleep()

	#active momentum reduction
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)

	move_cmd.angular.z = pi/16
	if Clockwise:
		move_cmd.angular.z = -pi/16

	#Move slowly until within 1.5 degrees - results in within .5 degrees on average
	while(abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) > 1.5 and abs(float(to_Angle)-(mybot.get_curr_angle()*360/(2*pi)+180)) < 358.5):
		cmd_vel.publish(move_cmd)
		r.sleep()

	#stop
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)
	print"Final angle is {}".format(mybot.get_curr_angle()*360/(2*pi)+180)


#Controls the omega and V of the turtblebot based on a desired input of
#x_dot & y_dot of a p attached to the robot a certain distance in front
#Takes the rate at which to move the pen (and direction) and a length of
#time to move the pen 
def Pen(xp, yp, time):
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	r = rospy.Rate(50) #update at 1/50 sec
	move_cmd = Twist()
	d_to_pen = .03
	print"Want Pen ({},{}) for {} sec".format(xp,yp,time)


	for i in range((time+1)*50): #time input for length of drive cmd_vel.publish(move_cmd)
		v = xp*math.cos(mybot.get_curr_angle()) + yp*math.sin(mybot.get_curr_angle())
		omega = -xp*math.sin(mybot.get_curr_angle())/d_to_pen + yp*math.cos(mybot.get_curr_angle())/d_to_pen
		move_cmd.angular.z = omega
		move_cmd.linear.x = v
		cmd_vel.publish(move_cmd)
		r.sleep() #sleep for .02

	move_cmd.angular.z = 0
	move_cmd.linear.x = 0
	cmd_vel.publish(move_cmd)


#Follows a squiggly tape line on the Robotics Lab floor
def SquigglyBack():
	Drive(.40)
	Turn(pi/2)
	Drive(.21)
	Turn(-pi/2)
	Drive(.31)
	Turn(-pi/2)
	Drive(.31)
	Turn(pi/2)
	Drive(.24)
	Turn(pi/2)
	Drive(.445)
	Turn(pi/2)
	Drive(.265)

	Drive(-.265)
	Turn(-pi/2)
	Drive(-.445)
	Turn(-pi/2)
	Drive(-.24)
	Turn(-pi/2)
	Drive(-.31)
	Turn(pi/2)
	Drive(-.31)
	Turn(pi/2)
	Drive(-.21)
	Turn(-pi/2)
	Drive(-.4)


#Drives in a square
def Square():
	TurnTo(0)
	Drive(.5)
	TurnTo(90)
	Drive(.5)
	TurnTo(180)
	Drive(.5)
	TurnTo(270)
	Drive(.5)
	TurnTo(0)

def main():
	rospy.init_node("Turn", anonymous=False)
	rate = rospy.Rate(50)
	

	SquigglyBack()

	# TurnTo(180)
	# Pen(.1, .1, 1)
	# Pen(-.1, .1, 1)
	# Pen(-.1, -.1, 1)
	# Pen(.1, -.1, 1)
	# TurnTo(180)

	rate.sleep()


mybot = RobotState()
main()

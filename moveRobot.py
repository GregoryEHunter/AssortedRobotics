import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# to take quarternion and spit out euler angle
from tf.transformations import euler_from_quaternion



def yaw_from_odom(msg):
	orientation_q = msg.pose.pose.orientation
	orientation_vec = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

	(roll, pitch, yaw) = euler_from_quaternion(orientation_vec)
	return (roll, pitch, yaw)


def position_from_odom(msg):
	pos = msg.pose.pose.position
	(x, y) = (pos.x, pos.y)
	return (x,y)


class RobotState:
	def __init__(self):
		self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)

		# keep track of angle and position
		self.angle = None
		self.position = None

		# bool for whether initial state was stored
		self.initial_set = False
		self.init_angle = None
		self.init_position = None

	# callback function
	def update_odom(self, msg):
		if self.initial_set is False:
			self.init_angle = yaw_from_odom(msg)
			self.init_position = position_from_odom(msg)
			self.initial_set = True
		else:
			self.angle = yaw_from_odom(msg)
			self.position = position_from_odom(msg)

			# debugging robot state
			# print("Initials:")
			# print(self.init_angle[2])
			# print(self.init_position)
			# print("Currents:")
			# print(self.angle[2])
			# print(self.position)

	def reset(self):
		self.initial_set = False
		self.init_angle = None
		self.init_position = None
		self.angle = None
		self.position = None

	def get_init_yaw(self):
		return self.init_angle[2]

	def get_curr_yaw(self):
		return self.angle[2]

	# def get_init_position(self):
	# 	return self.init_position

	# def get_init_angle(self):
	# 	return self.init_angle

	def get_curr_dist_x(self):
		return self.init_position[0]-self.position[0]

	def get_curr_dist_y(self):
		return self.init_position[1]-self.position[1]


# function that drives a specified distance and stops precisely
def drive(distance):
	# rospy.init_node('GoForward', anonymous=False)
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(20)

	move_cmd = Twist()

	# only linear x changes
	move_cmd.linear.y = 0
	move_cmd.linear.z = 0
	move_cmd.angular.x = 0
	move_cmd.angular.y = 0
	move_cmd.angular.z = 0

	speed = 0.2

	if float(distance) > 0:
		move_cmd.linear.x = speed
	else:
		move_cmd.linear.x = -speed

	t0 = rospy.Time.now().to_sec()
	curr_dist = 0.0

	while(abs(curr_dist) < abs(float(distance))):
		# publish velocity
		cmd_vel.publish(move_cmd)
		# find time now
		t1 = rospy.Time.now().to_sec()
		# calculatenew curr_dist
		curr_dist = speed * (t1-t0)

	# once out of the while loop, stop the robot	
	move_cmd.linear.x = 0
	cmd_vel.publish(move_cmd)
	mybot.reset()


# function that turn a specified angle in radians and stops precisely
def turn(angle):

	# defining math constants
	two_pi = 2.0*math.pi
	pi = math.pi
	half_pi = math.pi/2.0

	# rospy.init_node('Turn', anonymous=False)
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(50)
	#r = rospy.Rate(50) #update at 1/50 sec
	move_cmd = Twist()

	# only angular z changes
	move_cmd.linear.x = 0
	move_cmd.linear.y = 0
	move_cmd.linear.z = 0
	move_cmd.angular.x = 0
	move_cmd.angular.y = 0

	turn_speed = half_pi

	if float(angle) > 0:
		move_cmd.angular.z = turn_speed
		clockwise = False
	else:
		move_cmd.angular.z = -turn_speed
		clockwise = True

	angle = abs(angle)

	t0 = rospy.Time.now().to_sec()
	curr_angle_dist = 0.0;


	# Move quickly until within pi/8 radians of goal
	while(curr_angle_dist < angle and abs(angle-curr_angle_dist > pi/8.0)):
		# publish velocity
		cmd_vel.publish(move_cmd)
		# find time now
		t1 = rospy.Time.now().to_sec()
		# calculate new curr_dist
		curr_angle_dist1 = turn_speed * (t1-t0)
		curr_angle_dist = curr_angle_dist1

	# 0 out momentum	
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)



	# slow down turn speed when close to pi/8 radians
	turn_speed = pi/8.0
	t0 = rospy.Time.now().to_sec()

	if clockwise:
		move_cmd.angular.z = -turn_speed
	else:
		move_cmd.angular.z = turn_speed


	# Move slower until within pi/16 radians of goal
	while(curr_angle_dist < angle and abs(angle-curr_angle_dist) > pi/16.0):
		# publish velocity
		cmd_vel.publish(move_cmd)
		# find time now
		t1 = rospy.Time.now().to_sec()
		# calculate new curr_dist
		curr_angle_dist2 = turn_speed * (t1-t0)
		curr_angle_dist = curr_angle_dist1 + curr_angle_dist2


	# 0 out momentum	
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)



	# slow down turn speed when close to pi/16 radians
	turn_speed = pi/16.0
	t0 = rospy.Time.now().to_sec()

	if clockwise:
		move_cmd.angular.z = -turn_speed
	else:
		move_cmd.angular.z = turn_speed

	# Move slower until goal reached (use odom)
	while(abs(mybot.get_curr_yaw()-mybot.get_init_yaw()) < angle):
		# publish velocity
		cmd_vel.publish(move_cmd)
		# find time now
		t1 = rospy.Time.now().to_sec()
		# calculate new curr_dist
		curr_angle_dist3 = turn_speed * (t1-t0)
		curr_angle_dist = curr_angle_dist1 + curr_angle_dist2 + curr_angle_dist3

	# once out of the while loop, stop the robot	
	move_cmd.angular.z = 0
	cmd_vel.publish(move_cmd)
	mybot.reset()


def main():
	rospy.init_node('square', anonymous=False)

	# mybot = RobotState()
	# rospy.init_node("turn_to")
	# rate = rospy.Rate(20)
	# print("step1")
	drive(1)
	turn(math.pi/2)
	drive(1)
	turn(math.pi/2)
	drive(1)
	turn(math.pi/2)
	drive(1)
	turn(math.pi/2)

	# print("step2")
	# drive(-1)
	# turn(math.pi)
	# turn(math.pi)


	# cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# move_cmd = Twist()
	# move_cmd.linear.x = .2

	# for i in range(20):
	# 	cmd_vel.publish(move_cmd)
	# 	rate.sleep()
	# while not rospy.is_shutdown():
	# 	rate.sleep()


def stop():
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	rospy.loginfo("Stop turtlebot")
	cmd_vel.publish(Twist())
	rospy.sleep(1)

mybot = RobotState()
main()
stop()
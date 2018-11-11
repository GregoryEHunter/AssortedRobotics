import json
from pprint import pprint as pp
from angles import degrees_to_radians

import matplotlib.pyplot as plt
from math import cos, sin, pi

# Note: robot.txt file format should be a JSON blob

# open file put into python object
# with open("robot.txt") as file:
# 	for line in file:
# 		data = json.load(line)
contents = open("robot.txt", "r").read() 
data = [json.loads(str(item)) for item in contents.strip().split('\n')]

# init plot
fig = plt.figure()
ax = fig.add_subplot(111)

for i in range(len(data)):
	# load data into variables
	robot_x = data[i][str(i)]["position"]["x"]
	robot_y = data[i][str(i)]["position"]["y"]
	robot_theta = data[i][str(i)]["orientation"]["yaw"]

	scan_ranges = data[i][str(i)]["scan"]["ranges"]

	# plot the robot
	ax.scatter(robot_x, robot_y, color='red', marker= 'x')

	# create data arrays
	x = []
	y = []

	# iterate through ranges and plot
	for i in range(len(scan_ranges)):
		if scan_ranges[i] >= 0.03:
			x.append(robot_x + scan_ranges[i] * cos(robot_theta + degrees_to_radians(i)))
			y.append(robot_y + scan_ranges[i] * sin(robot_theta + degrees_to_radians(i)))

	ax.scatter(x, y, color='blue', marker= '.')

plt.show()
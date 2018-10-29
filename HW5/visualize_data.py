import json
from pprint import pprint as pp
from angles import degrees_to_radians

import matplotlib.pyplot as plt
from math import cos, sin, pi

# Note: robot.txt file format should be a JSON blob

# open file put into python object
with open("robot.txt") as file:
	data = json.load(file)

# load data into variables
robot_x = data["robot"]["position"]["x"]
robot_y = data["robot"]["position"]["y"]
robot_theta = data["robot"]["orientation"]["yaw"]

scan_ranges = data["robot"]["scan"]["ranges"]

# init plot
fig = plt.figure()
ax = fig.add_subplot(111)

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
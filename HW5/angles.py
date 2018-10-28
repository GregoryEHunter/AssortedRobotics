from math import pi

# compute angle in range [0, 2pi] from an angle
#  possibly outside that range
def rectify_angle_2pi(angle):
    while(angle < 0):
        angle += 2 * pi
    while(angle > 2 * pi):
        angle -= 2 * pi

    return angle

# compute angle in range [-pi, pi] from an angle
#  possibly outside that range
def rectify_angle_pi(angle):
    angle = rectify_angle_2pi(angle)

    if(angle > pi):
        angle -= 2 * pi

    return angle

# compute angle in radians from degrees
def degrees_to_radians(degrees):
	return degrees*(2*pi)/360
    

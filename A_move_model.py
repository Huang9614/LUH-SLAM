from math import sin, cos, pi
import matplotlib.pyplot as plt
from lego_robot import *

def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width, scanner_displacement):

    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        theta = old_pose[2]
        l = motor_ticks[0] * ticks_to_mm
        theta = (theta)%(2*pi)
        x = old_pose[0] + l*cos(theta)
        y = old_pose[1] + l*sin(theta)

        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        theta = old_pose[2]
        w = robot_width
        l = motor_ticks[0] * ticks_to_mm
        r = motor_ticks[1] * ticks_to_mm
        alpha = (r-l)/w
        R = l/alpha
        
        Xc = (old_pose[0] - scanner_displacement) - (R+w/2)*sin(theta) 
        Yc = (old_pose[1] - scanner_displacement) - (R+w/2)*(-cos(theta))
        
        theta = (theta + alpha)%(2*pi)
        
        x = Xc + (R+w/2)*sin(theta) + scanner_displacement
        y = Yc + (R+w/2)*(-cos(theta)) + scanner_displacement
        
        return (x, y, theta)


scanner_displacement = 30.0
# Empirically derived distance between scanner and assumed center of robot.
ticks_to_mm = 0.349
# Measured width of the robot (wheel gauge), in mm.
robot_width = 173.0

# Read data.
logfile = LegoLogfile()
logfile.read("robot4_motors.txt")

# init. pos 1: Start at origin (0,0), looking along x axis (alpha = 0).
#pose = (0.0, 0.0, 0.0)
# init. pos 2: Measured start position.
pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

# Loop over all motor tick records generate filtered position list.
filtered = []
for ticks in logfile.motor_ticks:
    pose = filter_step(pose, ticks, ticks_to_mm, robot_width,scanner_displacement)
    filtered.append(pose)

# Draw result.
for pose in filtered:
    print(pose)
    plt.plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
plt.show()
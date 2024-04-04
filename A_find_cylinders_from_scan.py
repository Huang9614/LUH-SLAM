from lego_robot import *
from math import sin, cos

def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in range(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    
    for i in range(len(scan_derivative)):
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list
        if scan_derivative[i] <= -jump:
            on_cylinder = True
            sum_ray, sum_depth, rays = 0.0, 0.0, 0

        if on_cylinder:
            if abs(scan_derivative[i]) <= jump: # Practically constant
                sum_ray += i
                sum_depth += scan[i]
                rays += 1
            elif scan_derivative[i] >= jump:
                on_cylinder = False
                cylinder_list.append( (sum_ray/rays, sum_depth/rays) )


    return cylinder_list

def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:
        # --->>> Insert here the conversion from polar to Cartesian coordinates.
        # c is a tuple (beam_index, range).
        # For converting the beam index to an angle, use
        # LegoLogfile.beam_index_to_angle(beam_index)
        LiDar_index = c[0]
        depth = c[1]
        angle = LegoLogfile.beam_index_to_angle(LiDar_index)
        x = (depth+cylinder_offset) * cos(angle)
        y = (depth+cylinder_offset) * sin(angle)
        result.append( (x,y) ) # Replace this by your (x,y)
    return result

minimum_valid_distance = 20.0
depth_jump = 100.0
cylinder_offset = 90.0

# Read the logfile which contains all scans.
logfile = LegoLogfile()
logfile.read("robot4_scan.txt")

# Write a result file containing all cylinder records.
# Format is: D C x[in mm] y[in mm] ...
# With zero or more points.
# Note "D C" is also written for otherwise empty lines (no
# cylinders in scan)
with open("cylinders.txt", "w") as file:

    for scan in logfile.scan_data:
        # Find cylinders.
        der = compute_derivative(scan, minimum_valid_distance)
        cylinders = find_cylinders(scan, der, depth_jump,
                                    minimum_valid_distance)
        cartesian_cylinders = compute_cartesian_coordinates(cylinders,
                                                            cylinder_offset)
        # Write to file.
        file.write("D C ")
        for c in cartesian_cylinders:
            file.write(f"{c[0]:.1f} {c[1]:.1f} ")
        file.write("\n")
            
            




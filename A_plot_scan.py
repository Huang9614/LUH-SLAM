import matplotlib.pyplot as plt
from lego_robot import *

# Read the logfile which contains all scans.
logfile = LegoLogfile()
logfile.read("robot4_scan.txt")

# Plot one scan.
plt.plot(logfile.scan_data[8])
plt.show()

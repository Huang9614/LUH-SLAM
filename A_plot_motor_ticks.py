import matplotlib.pyplot as plt

f = open("robot4_motors.txt")

left_list = []
right_list = []
for l in f:
    sp = l.split()
    left_list.append(int(sp[2]))
    right_list.append(int(sp[6]))

plt.plot(left_list)
plt.plot(right_list)
plt.show()
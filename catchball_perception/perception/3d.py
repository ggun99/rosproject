import matplotlib as mpl
import matplotlib.pyplot as plt
import math
f1 = open('/home/airlab/ros2_ws/position2.txt', 'r')
coordinates = f1.readlines()
coordinates = [x.strip("\n[]") for x in coordinates]
coordinates = [y.split(' ') for y in coordinates]
r1 = []
t1 = []
p1 = []
for i in range(0,20):
    coordinates[i][0] = float(coordinates[i][0])
    coordinates[i][1] = float(coordinates[i][1])
    coordinates[i][2] = float(coordinates[i][2])

    r1.append(coordinates[i][0])
    t1.append(coordinates[i][1])
    p1.append(coordinates[i][2])
# GRAPH
mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure(figsize=(10,5))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(r1, t1, p1,label='Ball Position', c='blue')
ax.legend(loc='center left', bbox_to_anchor=(1.1, 0.5))
title_font = {
    'fontsize': 16,
    'fontweight': 'bold'
}
# Set the limits of the plot
# ax.set_xlim([50, 75])
# ax.set_ylim([100, 220.0])
# ax.set_zlim([1200, 1600])
# ax.set_xlabel('x axis(mm)')
ax.set_ylabel('y axis(mm)')
ax.set_zlabel('z axis(mm)')
ax.view_init(20,180)
plt.title('Ball Trajectory', fontdict=title_font, loc='center', pad = 20)
plt.show()
import numpy as np
import matplotlib.pyplot as plt

f4 = open("/home/airlab/ros2_ws/src/cv_get_ball_position/cv_get_ball_position/position.txt","r")
pos = f4.readlines()
pos = [x.strip("\n[]") for x in pos]
pos = [y.split(' ') for y in pos]
parray = np.array(pos)
x = parray[:,0]
y = parray[:,1]
z = parray[:,2]
x = x.astype(np.float)
y = y.astype(np.float)
z = z.astype(np.float)

fig = plt.figure()
ax = fig.add_subplot(111, '3d')
ax.plot(x,y,z,'ro')
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.legend()

plt.show()
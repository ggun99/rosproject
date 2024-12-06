import matplotlib.pyplot as plt
import numpy as np

# file = np.loadtxt("/home/airlab/ros2_ws/center_with_time.txt", delimiter=" ",unpack = False)
# print(file.shape)

f1 = open('/home/airlab/center_with_time.txt', 'r')
coordinates = f1.readlines()
coordinates = [x.strip("\n[]") for x in coordinates]
coordinates = [y.split(' ') for y in coordinates]
#number = np.split(108:1:108)

coordinates = np.array(coordinates)
coordinates = coordinates.astype(np.float64)
#print(coordinates)

x = np.arange(0,216)
y = sum(coordinates[:,2])
y = y/216
print(y)
# plt.plot(x,y,'r')
# plt.show()
#time = file[:,2]

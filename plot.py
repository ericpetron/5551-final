import matplotlib.pyplot as plt
import numpy as n

point1 = n.loadtxt('results/file1.txt', delimiter=" ")
point2 = n.loadtxt('results/file2.txt', delimiter=" ")
point3 = n.loadtxt('results/file3.txt', delimiter=" ")
# points = n.concatenate((point1,point2,point3))
fig = plt.figure()
ax = fig.add_subplot(projection="3d")
ax.scatter(point1[:, 0], point1[:, 1], zs=point1[:, 2], c='red')

ax.scatter(point2[:, 0], point2[:, 1], zs=point2[:, 2], c='blue')
ax.scatter(point3[:, 0], point3[:, 1], zs=point3[:, 2], c='green')

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_aspect("equal")
plt.show()
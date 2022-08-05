import matplotlib.pyplot as plt
import numpy as np

filename = "experiment_data/012522-22:25:39-data.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=2)

#cf1_actual_position = data[:, 18:21]
#human_1_position = data[:,25:28]

cf1_actual_position = data[:, 22:25]
human_1_position = data[:, 29:32]
human_2_position = data[:,33:36]

#waypoints_cf1 = []
#waypoints_cf2 = []
#for i in range(data.shape[0]):
#    waypoints_cf1.append(list(data[i, 0:3]))
#    waypoints_cf2.append(list(data[i, 13:16]))

r = 0.3
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = r*np.cos(u)*np.sin(v)
y = r*np.sin(u)*np.sin(v)
z = r*np.cos(v) + 1

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(cf1_actual_position[:,0],cf1_actual_position[:,1], cf1_actual_position[:,2], label="CF1")
ax.scatter(cf1_actual_position[0,0],cf1_actual_position[0,1],cf1_actual_position[0,2],label="start position")
ax.plot(human_1_position[:,0],human_1_position[:,1], human_1_position[:,2], label="human-1")
ax.plot(human_2_position[:,0],human_2_position[:,1], human_2_position[:,2], label="human-2")

#ax.plot_surface(x, y, z, cmap=plt.cm.YlGnBu_r)
ax.set_xlim((-3,3))
ax.set_ylim((-3,3))
ax.set_zlim((0,3))
#ax.plot(data[:,13],data[:,14], data[:,15] ,label="CF2")
plt.legend()
plt.show()

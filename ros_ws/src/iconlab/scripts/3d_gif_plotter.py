import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


filename = "experiment_data/030922-18:23:34-data.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=2)

timestamps = data[:, 0]
timestamps -= timestamps[0]

cf1_actual_position = data[:, 22:25]
human_1_position = data[:, 29:32]
human_2_position = data[:,33:36]

def init_animation():
    cf1_line.set_data([],[])
    human1_line.set_data([],[])
    human2_line.set_data([],[])
    return cf1_line, human1_line, human2_line

def update_animation(frame):

    cf1_line.set_data(cf1_actual_position[0:frame, 0],
                      cf1_actual_position[0:frame, 1])

    cf1_line.set_3d_properties(cf1_actual_position[0:frame, 2])

    human1_line.set_data(human_1_position[0:frame, 0],
                      human_1_position[0:frame, 1])

    human1_line.set_3d_properties(human_1_position[0:frame, 2])

    human2_line.set_data(human_2_position[0:frame, 0],
                      human_2_position[0:frame, 1])

    human2_line.set_3d_properties(human_2_position[0:frame, 2])

    return cf1_line, human1_line, human2_line


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

ax.set_xlim3d([-2.0, 2.0])
ax.set_xlabel('X')
ax.set_ylim3d([-2.0, 2.0])
ax.set_ylabel('Y')
ax.set_zlim3d([-2.0, 2.0])
ax.set_zlabel('Z')

cf1_line = ax.plot([],[],[], label="CF1 Position")[0]
human1_line = ax.plot([],[],[])[0]
human2_line = ax.plot([],[],[])[0]

line_ani = animation.FuncAnimation(fig,
                                   update_animation,
                                   init_func=init_animation,
                                   frames=len(timestamps),
                                   interval=50,
                                   blit=False)

plt.show()

line_ani.save('animation.gif', writer='imagemagick', fps=10)

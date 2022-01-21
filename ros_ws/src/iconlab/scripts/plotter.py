import matplotlib.pyplot as plt
import numpy as np

filename = "experiment_data/012122-14:52:14-data.csv"

data = np.genfromtxt(filename, delimiter=',', skip_header=2)

timestamps = data[:, 0]
timestamps -= timestamps[0]

cf1_desired_position = data[:, 1:4]
cf1_desired_quatrn = data[:, 4:8]
cf1_actual_position = data[:, 8:11]
cf1_actual_quatrn = data[:, 11:15]

plt.figure()
plt.subplot(311)
plt.plot(timestamps, cf1_actual_position[:, 0], label="CF1 X Actual")
plt.plot(timestamps, cf1_desired_position[:, 0], label="CF1 X Desired")
plt.legend()
plt.subplot(312)
plt.plot(timestamps, cf1_actual_position[:, 1], label="CF1 Y Actual")
plt.plot(timestamps, cf1_desired_position[:, 1], label="CF1 Y Desired")
plt.legend()
plt.subplot(313)
plt.plot(timestamps, cf1_actual_position[:, 2], label="CF1 Z Actual")
plt.plot(timestamps, cf1_desired_position[:, 2], label="CF1 Z Desired")
plt.legend()

plt.show()

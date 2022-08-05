#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from pycrazyswarm import *
import datetime
import csv
import time
import numpy as np

import julia
jl = julia.Julia(compiled_modules=False)
from julia import Main

# Enable or disable using Julia MPC algorithm
USE_JULIA = True

# Assemble filename for logged data
datetimeString = datetime.datetime.now().strftime("%m%d%y-%H:%M:%S")
csv_filename = "experiment_data/" + datetimeString + "-2drones.csv"

# Enable or disable data logging
LOG_DATA = True

TAKEOFF_Z = 1.0
TAKEOFF_DURATION = 3.0

# Used to tune aggresiveness of low-level controller
GOTO_DURATION = 1.6

# Defining takeoff and experiment start position
cf1_takeoff_pos = [0.0, 0.0, 1.0]
cf1_start_pos = [-1.0, -1.5, 1.0]
cf2_takeoff_pos = [-0.5, 0.0, 0.5]
cf2_start_pos = [-1.5, -1.0, 1.0]

# Import waypoints from csv file
csvfilename = "Waypoints_two_quad.csv"
data = np.genfromtxt(csvfilename, delimiter=',').T
data[:,0] -= 1.5
data[:,1] -= 1.5
data[:,13] -= 1.5
data[:,14] -= 1.5
waypoints_cf1 = []
waypoints_cf2 = []
for i in range(data.shape[0]):
    waypoints_cf1.append(list(data[i, 0:3]))
    waypoints_cf2.append(list(data[i, 13:16]))

def perform_experiment():

    # Wait for button press for take off
    raw_input("##### Press Enter to Take Off #####")

    cf1.takeoff(TAKEOFF_Z, TAKEOFF_DURATION)
    cf2.takeoff(TAKEOFF_Z, TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    cf1.goTo(cf1_start_pos, yaw=0.0, duration=2.0)
    cf2.goTo(cf2_start_pos, yaw=0.0, duration=2.0)
    timeHelper.sleep(2.0)

    # Wait for button press to begin experiment
    raw_input("##### Press Enter to Begin Experiment #####")

    for i in range(len(waypoints_cf1)):

        #print("Desired Position: " + str(waypoints[i]))

        cf1.goTo(waypoints_cf1[i], yaw=0.0, duration=GOTO_DURATION)
        cf2.goTo(waypoints_cf2[i], yaw=0.0, duration=GOTO_DURATION)

        try:
            (pos_cf1,rot_cf1) = listener.lookupTransform('/world', '/cf1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        try:
            (pos_cf2,rot_cf2) = listener.lookupTransform('/world', '/cf2', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # Only reaches here if a /tf message was received
        print("CF1 Position: " + str(pos_cf1))
        print("CF1 Rotation: " + str(rot_cf1))
        print("CF2 Position: " + str(pos_cf2))
        print("CF2 Rotation: " + str(rot_cf2))

        x_update_cf1 = [pos_cf1[0], pos_cf1[1], pos_cf1[2], rot_cf1[3], rot_cf1[0], rot_cf1[1], rot_cf1[2]]
        x_update_cf2 = [pos_cf2[0], pos_cf2[1], pos_cf2[2], rot_cf2[3], rot_cf2[0], rot_cf2[1], rot_cf2[2]]

        if LOG_DATA:
            timestampString = str(time.time())
            #csvwriter.writerow([timestampString] + xd + x_update + xd_actual)
            csvwriter.writerow([timestampString] +
                               waypoints_cf1[i] + x_update_cf1 +
                               waypoints_cf2[i] + x_update_cf2)

        rate.sleep()

    raw_input("##### Press Enter to Go Back to Origin #####")

    cf1.goTo(cf1_takeoff_pos, yaw=0.0, duration=3.0)
    cf2.goTo(cf2_takeoff_pos, yaw=0.0, duration=3.0)
    timeHelper.sleep(4.0)

    cf1.land(targetHeight=0.05, duration=3.0)
    cf2.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4.0)




if __name__ == '__main__':
    #rospy.init_node('tf_listener')

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    num_cfs = len(swarm.allcfs.crazyflies)

    cf1 = swarm.allcfs.crazyflies[0]
    cf2 = swarm.allcfs.crazyflies[1]

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    if LOG_DATA:
        print("### Logging data to file: " + csv_filename)
        csvfile = open(csv_filename, 'w')
        csvwriter = csv.writer(csvfile, delimiter=',')

        csvwriter.writerow(['# CFs', str(num_cfs)])
        csvwriter.writerow(["Timestamp [s]"] + num_cfs*["TODO (disregard)"])

    try:
        perform_experiment()

    except Exception as e:
        print ("##### Python exception occurred! Returning to start location and landing #####")
        cf1.goTo(cf1_takeoff_pos, yaw=0.0, duration=3.0)
        cf2.goTo(cf2_takeoff_pos, yaw=0.0, duration=3.0)
        timeHelper.sleep(4.0)
        cf1.land(targetHeight=0.05, duration=3.0)
        cf2.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)
        raise(e)

    except KeyboardInterrupt:
        print ("##### KeyboardInterrupt detected. Landing all CFs  #####")
        cf1.land(targetHeight=0.05, duration=3.0)
        cf2.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)

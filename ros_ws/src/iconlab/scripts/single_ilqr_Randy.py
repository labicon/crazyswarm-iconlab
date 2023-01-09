#make sure decentralized is in PYTHONPATH
from time import perf_counter as pc
import warnings

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from decentralized import split_agents, plot_solve
import decentralized as dec
# import pocketknives

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from pycrazyswarm import *
from pycrazyswarm import crazyflie
import datetime
import csv
import time

# Assemble filename for logged data
datetimeString = datetime.datetime.now().strftime("%m%d%y-%H:%M:%S")
csv_filename = "experiment_data/" + datetimeString + "-data.csv"

# Enable or disable data logging
LOG_DATA = False

TAKEOFF_Z = 1.0
TAKEOFF_DURATION = 3.0

# Used to tune aggresiveness of low-level controller
GOTO_DURATION = 1.75

#Using drone No. 10:
# Defining takeoff and experiment start position
cf10_takeoff_pos = [0.0, 0.0, 1.0]
cf10_start_pos = [-2.0, 0.0, 1.0]

"""
The states of the quadcopter are: px, py ,pz, vx, vy, vz
"""
radius = 0.5

def perform_experiment():
    
    # Wait for button press for take off
    input("##### Press Enter to Take Off #####")

    cf10.takeoff(TAKEOFF_Z, TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    cf10.goTo(cf10_start_pos, yaw=0.0, duration=2.0)
    timeHelper.sleep(2.0)

    # Wait for button press to begin experiment
    input("##### Press Enter to Begin Experiment #####")
    
    #Decentrzlied iLQR code here:
    cf10_goal_pos = [3.0, 3.0, 1.5]

    x = np.hstack([cf10_start_pos, np.zeros(3)])
    x_goal = np.hstack([cf10_goal_pos, np.zeros((3))])

    n_agents = 1
    n_states = 6
    n_controls = 3
    n_dim = 3

    dt = 0.1
    N = 10
    tol = 1e-3
    ids = [100 + i for i in range(n_agents)]
    model = dec.QuadcopterDynamics6D
    dynamics = dec.MultiDynamicalModel([model(dt, id_) for id_ in ids])
    Q = 1.0 * np.diag([10., 10., 10., 10., 10., 10.])
    Qf = 1000 * np.eye(Q.shape[0])
    R = np.eye(3)

    # radius = ENERGY / 20
    radius = 0.5
    x_dims = [n_states] * n_agents
    u_dims = [n_controls] * n_agents
    
    goal_costs = [dec.ReferenceCost(x_goal_i, Q.copy(), R.copy(), Qf.copy(), id_) 
                for x_goal_i, id_ in zip(split_agents(x_goal.T, x_dims), ids)]
    prox_cost = dec.ProximityCost(x_dims, radius, n_dim)
    game_cost = dec.GameCost(goal_costs, prox_cost)

    prob = dec.ilqrProblem(dynamics, game_cost)
    step_size = 5
    n_d = 3
    
    for i in range(LOOP_ITERS):
    
        X, U, J = dec.solve_rhc(
                        prob, x, N, radius,
                        centralized=True,
                        n_d=n_d,
                        step_size=step_size, 
                        dist_converge=0.1,
                        verbose=True,
                        t_kill=step_size*dt,
                        t_diverge=None,
                    )

        # print("shape of array X is :" + str(X.shape))
        xd = X[step_size,0:3] #x, y, z coordinates from the solved trajectory X
        
        cf10.goTo(xd, yaw=0.0, duration=GOTO_DURATION)
        
        """
        listener.lookupTransform returns two lists: the 1st list are (x,y,z) linear transformations
        of the child frame relative to the parent frame, and the second list are (x,y,z,w) quarternion 
        required to rotate from the parent oreintation to the child oreintation
        """
        pos_cf, _ = listener.lookupTransform('/world', '/cf10', rospy.Time(0))
        pos_cf = np.array(pos_cf)
        
        # try:
        #     (pos_cf1,rot_cf1) = listener.lookupTransform('/world', '/cf1', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        # Only reaches here if a /tf message was received
        print("CF Position: " + str(pos_cf))
        
        
        x_prev = x.copy()
        dV = (pos_cf - x_prev[0:3]) / dt  #this is a vector of length 3
        x = np.hstack([pos_cf, dV])

        if LOG_DATA:
                timestampString = str(time.time())
                csvwriter.writerow([timestampString] + pos_cf)

        rate.sleep()

    input("##### Press Enter to Go Back to Origin #####")

    cf10.goTo(cf10_takeoff_pos, yaw=0.0, duration=3.0)
    timeHelper.sleep(4.0)

    cf10.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4.0)

    

if __name__ == '__main__':
    #rospy.init_node('tf_listener')

    LOOP_ITERS = 100

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    num_cfs = len(swarm.allcfs.crazyflies)

    cf10 = swarm.allcfs.crazyflies[0]
    # cf10 = crazyflie.Crazyflie(10,cf10_start_pos,tf).setGroupMask(0) #this is right?

    listener = tf.TransformListener()

    rate = rospy.Rate(5.0)

    if LOG_DATA:
        print("### Logging data to file: " + csv_filename)
        csvfile = open(csv_filename, 'w')
        csvwriter = csv.writer(csvfile, delimiter=',')

        csvwriter.writerow(['# CFs', str(num_cfs)])
        csvwriter.writerow(["Timestamp [s]"] + num_cfs*["x_d", "y_d", "z_d", " x", "y", "z", "qw", "qx", "qy", "qz"])

    try:
        perform_experiment()


    except Exception as e:
        print ("##### Python exception occurred! Returning to start location and landing #####")
        cf10.goTo(cf10_takeoff_pos, yaw=0.0, duration=3.0)
        timeHelper.sleep(4.0)
        cf10.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)
        raise(e)

    except KeyboardInterrupt:
        print ("##### KeyboardInterrupt detected. Landing all CFs  #####")
        cf10.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)

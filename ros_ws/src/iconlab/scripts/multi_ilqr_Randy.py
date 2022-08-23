#make sure decentralized is in PYTHONPATH
from time import perf_counter as pc
import warnings

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from decentralized import split_agents, plot_solve
import decentralized as dec
import pocketknives

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from pycrazyswarm import *
import datetime
import csv
import time

# Assemble filename for logged data
datetimeString = datetime.datetime.now().strftime("%m%d%y-%H:%M:%S")
csv_filename = "experiment_data/" + datetimeString + "-data.csv"

# Enable or disable data logging
LOG_DATA = True

TAKEOFF_Z = 1.0
TAKEOFF_DURATION = 3.0

# Used to tune aggresiveness of low-level controller
GOTO_DURATION = 1.75

# Defining takeoff positions for each agent:
cf1_takeoff_pos = [0.0, 0.0, 1.0]
cf2_takeoff_pos = [0.5, 0.5, 1.0]
cf3_takeoff_pos = [-0.5, -0.5, 1.0]

#now we randomize the start_pos and goal_pos for each agent:
n_agents = 3
n_states = 6
n_controls = 3
ENERGY = 10.0
n_d = 3
x0, x_goal = dec.random_setup(
    n_agents, n_states, 
    is_rotation=False, 
    rel_dist=2.0, 
    var=1.0, 
    n_d=n_d, 
    random=True
)

x = x0

cf1_start_pos = x0[0:6]
cf2_start_pos = x0[6:12]
cf3_start_pos = x0[12:18]

cf1_goal_pos = x_goal[0:6]
cf2_goal_pos = x_goal[6:12]
cf3_goal_pos = x_goal[12:18]

x_dims = [n_states] * n_agents
u_dims = [n_controls] * n_agents

"""
The states of each quadcopter are: px, py ,pz, vx, vy, vz
"""
radius = 0.5

def perform_experiment():
    
    # Wait for button press for take off
    input("##### Press Enter to Take Off #####")
    
    swarm.allcfs.takeoff(targetHeight=TAKEOFF_Z, duration=1.0+TAKEOFF_Z)
    timeHelper.sleep(1.5+TAKEOFF_Z)
    
    # Wait for button press to begin experiment
    input("##### Press Enter to Begin Experiment #####")
    
    #Decentrzlied iLQR code here:
    
    Q = np.diag([50.0, 50.0, 50.0, 0.0, 0.0, 0.0,])
    Qf = 100 * np.eye(Q.shape[0])
    R = np.eye(3)

    n_agents = 1
    n_states = 6
    n_controls = 3

    dt = 0.1
    tol = 1e-3
    ids = [100 + i for i in range(n_agents)]

    model = dec.QuadcopterDynamics6D #6-dimensional quadcopter dynamics

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
    prox_cost = dec.ProximityCost(x_dims, radius)
    game_cost = dec.GameCost(goal_costs, prox_cost)

    prob = dec.ilqrProblem(dynamics, game_cost)
    step_size = 2

    n_d = 3
    N = 50
    for i in range(LOOP_ITERS):
    
        X, U, J = dec.solve_rhc(
                        prob, x, N, radius,
                        centralized=True,
                        n_d=n_d,
                        step_size=step_size, 
                        dist_converge=0.1,
                        verbose=True,
                        t_kill=step_size*dt,
                        t_diverge=N*dt
                    )

        # print("shape of array X is :" + str(X.shape))
        
        xd1 = X[step_size,0:3] #x, y, z coordinates from the solved trajectory X
        cf1.goTo(xd1, yaw=0.0, duration=GOTO_DURATION)

        xd2 = X[step_size,6:9] #x, y, z coordinates from the solved trajectory X
        cf2.goTo(xd2, yaw=0.0, duration=GOTO_DURATION)

        xd3 = X[step_size,12:15] #x, y, z coordinates from the solved trajectory X
        cf3.goTo(xd3, yaw=0.0, duration=GOTO_DURATION)
    

        """
        listener.lookupTransform returns two lists: the 1st list are (x,y,z) linear transformations
        of the child frame relative to the parent frame, and the second list are (x,y,z,w) quarternion 
        required to rotate from the parent oreintation to the child oreintation
        """
        pos_cf1, _ = listener.lookupTransform('/world', '/cf1', rospy.Time(0))
        pos_cf1 = np.array(pos_cf1)

        pos_cf2, _ = listener.lookupTransform('/world', '/cf2', rospy.Time(0))
        pos_cf2 = np.array(pos_cf2)

        pos_cf3, _ = listener.lookupTransform('/world', '/cf3', rospy.Time(0))
        pos_cf3 = np.array(pos_cf3)


        # Only reaches here if a /tf message was received
        print("CF1 Position: " + str(pos_cf1))
        print("CF2 Position: " + str(pos_cf2))
        print("CF3 Position: " + str(pos_cf3))
        
        x_prev = x.copy()
        # x[:,0:3] = x_update[0:3]
        
        dV1 = (pos_cf1-x_prev[0:3])/dt
        dV2 = (pos_cf1-x_prev[6:9])/dt
        dV3 = (pos_cf1-x_prev[12:15])/dt

        x = np.hstack([pos_cf1, dV1, pos_cf2, dV2, pos_cf3, dV3])

        if LOG_DATA:
                timestampString = str(time.time())
                csvwriter.writerow([timestampString] +  x)

        rate.sleep()

    input("##### Press Enter to Go Back to Origin #####")

    cf1.goTo(cf1_takeoff_pos, yaw=0.0, duration=3.0)
    timeHelper.sleep(4.0)
    cf1.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4.0)

    cf2.goTo(cf2_takeoff_pos, yaw=0.0, duration=3.0)
    timeHelper.sleep(4.0)
    cf2.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4.0)

    cf3.goTo(cf3_takeoff_pos, yaw=0.0, duration=3.0)
    timeHelper.sleep(4.0)
    cf3.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4.0)

    
if __name__ == '__main__':
    #rospy.init_node('tf_listener')

    LOOP_ITERS = 100

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    num_cfs = len(swarm.allcfs.crazyflies)

    cf1 = swarm.allcfs.crazyflies[0]
    cf2 = swarm.allcfs.crazyflies[1]
    cf3 = swarm.allcfs.crazyflies[2]

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
        cf1.goTo(cf1_takeoff_pos, yaw=0.0, duration=3.0)
        timeHelper.sleep(4.0)
        cf1.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)

        cf2.goTo(cf2_takeoff_pos, yaw=0.0, duration=3.0)
        timeHelper.sleep(4.0)
        cf2.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)

        cf3.goTo(cf3_takeoff_pos, yaw=0.0, duration=3.0)
        timeHelper.sleep(4.0)
        cf3.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)


        raise(e)

    except KeyboardInterrupt:
        print ("##### KeyboardInterrupt detected. Landing all CFs  #####")

        cf1.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)

        cf2.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)

        cf3.land(targetHeight=0.05, duration=3.0)
        timeHelper.sleep(4.0)


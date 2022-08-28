#make sure decentralized is in PYTHONPATH
from time import perf_counter as pc
import warnings

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from decentralized import split_agents, plot_solve, pos_mask
import decentralized as dec
import pocketknives

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


    
plt.ion()

# Assemble filename for logged data
datetimeString = datetime.datetime.now().strftime("%m%d%y-%H:%M:%S")
csv_filename = "experiment_data/" + datetimeString + "-data.csv"

# Enable or disable data logging
LOG_DATA = False

TAKEOFF_Z = 1.0
TAKEOFF_DURATION = 3.5

# Used to tune aggresiveness of low-level controller
GOTO_DURATION = 4.0
GOHOME_DURATION = 6.0

FLY = True

# Defining takeoff and experiment start position
start_pos_drone1 = [0., 0., 1.0]
start_pos_drone2 = [0.5, 0.5, 1.0]
start_pos_drone3 = [-0.5, -0.5, 1.0]

goal_pos_drone1 = [3.0, 3.0, 3.0]
goal_pos_drone2 = [-3.0, 3.0, 2.5]
goal_pos_drone3 = [2.5, 1.5, 3.1]

start_pos_list = [start_pos_drone1,start_pos_drone2,start_pos_drone3]
goal_pos_list = [goal_pos_drone1,goal_pos_drone2 , goal_pos_drone3]


"""
The states of the quadcopter are: px, py ,pz, vx, vy, vz
"""
def perform_experiment(centralized=False):

    fig = plt.figure()
    
    # Wait for button press for take off
    input("##### Press Enter to Take Off #####")

    allcfs = swarm.allcfs
    if FLY:
        allcfs.takeoff(targetHeight=TAKEOFF_Z, duration=1.0+TAKEOFF_Z)
        timeHelper.sleep(TAKEOFF_DURATION)
        allcfs.goToAbsolute(start_pos_list, duration=GOTO_DURATION)

    # Wait for button press to begin experiment
    input("##### Press Enter to Begin Experiment #####")
    
    n_agents = 3
    n_states = 6
    n_controls = 3
    n_dim = 3
        
    x = np.hstack([start_pos_list,np.zeros((n_agents,3))]).flatten() 
    x_goal = np.hstack([goal_pos_list,np.zeros((n_agents,3))]).flatten()

    dt = 0.1
    N = 10
    ids = [100 + i for i in range(n_agents)]
    model = dec.QuadcopterDynamics6D
    dynamics = dec.MultiDynamicalModel([model(dt, id_) for id_ in ids])
    Q = 1.0 * np.diag([10., 10., 10., 10., 10., 10.])
    Qf = 100 * np.eye(Q.shape[0])
    R = np.diag([0., 1., 1.])

    radius = 0.5
    x_dims = [n_states] * n_agents
    # u_dims = [n_controls] * n_agents
    
    goal_costs = [dec.ReferenceCost(x_goal_i, Q.copy(), R.copy(), Qf.copy(), id_) 
                for x_goal_i, id_ in zip(split_agents(x_goal.T, x_dims), ids)]
    prox_cost = dec.ProximityCost(x_dims, radius, n_dim)
    game_cost = dec.GameCost(goal_costs, prox_cost)

    prob = dec.ilqrProblem(dynamics, game_cost)
    centralized_solver = dec.ilqrSolver(prob, N)
    
    xi = x.reshape(1, -1)
    U = np.zeros((N, n_controls*n_agents))
    ids = prob.ids.copy()

    step_size = 1
    
    X_full = np.zeros((0, n_states*n_agents))
    U_full = np.zeros((0, n_controls*n_agents))
    X = np.tile(xi,(N+1, 1))
    
    while not np.all(dec.distance_to_goal(xi,x_goal,n_agents,n_states,3) <= 0.2):

        # How to feed state back into decentralization?
        #  1. Only decentralize at the current state.
        #  2. Offset the predicted trajectory by the current state.
        # Go with 2. and monitor the difference between the algorithm and VICON.
        if centralized:
            X, U, J, _ = dec.solve_centralized(
                centralized_solver, xi, U, ids, verbose=False
            )
        else:
            X, U, J, _ = dec.solve_decentralized(
                prob, X, U, radius, pool=None, verbose=False
                )
        
        # Record which steps were taken for plotting.
        X_full = np.r_[X_full, X[:step_size]]
        U_full = np.r_[U_full, U[:step_size]]

        # Seed the next iteration with the last state.
        X = np.r_[X[step_size:], np.tile(X[-1], (step_size, 1))]
        U = np.r_[U[step_size:], np.zeros((step_size, n_controls*n_agents))]

        # x, y, z coordinates from the solved trajectory X.
        xd = X[step_size].reshape(n_agents, n_states)[:, :3]
        if FLY:
            swarm.allcfs.goToAbsolute(xd, duration=GOTO_DURATION)
   
        pos_cfs = [cf.position() for cf in swarm.allcfs.crazyflies]
        vel_cfs = [cf.velocity() for cf in swarm.allcfs.crazyflies]

        # x_prev = xi.copy()
        # dV = (pos_cf - x_prev[0:3]) / dt
        # x = np.hstack([pos_cf, dV])
        xi = np.hstack([pos_cfs, vel_cfs]).flatten()

        state_error = np.abs(X[0] - xi)
        print(f"CF states: \n{xi.reshape(n_agents, n_states)}\n")
        print(f"Predicted state error: {state_error}")

        # # Offset the currently predicted states with the actual ones.
        # X[:, pos_mask(x_dims, 3)] += xi[pos_mask(x_dims, 3)]
        # # TODO: see if this velocity makes sense here.
        # X[0, ~pos_mask(x_dims, 3)] = xi[~pos_mask(x_dims, 3)]

        plt.clf()
        plot_solve(X_full, J, x_goal, x_dims, n_d=3)
        fig.canvas.draw()
        plt.pause(0.001)

        if LOG_DATA:
            timestampString = str(time.time())
            csvwriter.writerow([timestampString] + pos_cfs + vel_cfs)

        rate.sleep()

    input("##### Press Enter to Go Back to Origin #####")
    
    if FLY:
        swarm.allcfs.goToAbsolute(start_pos_list, duration=GOTO_DURATION)
        timeHelper.sleep(4.0)

        swarm.allcfs.land(targetHeight=0.05, duration=GOTO_DURATION)
        timeHelper.sleep(4.0)


if __name__ == '__main__':
    #rospy.init_node('tf_listener')

    swarm = Crazyswarm()
    # swarm.allcfs.setParam("colAv/enable", 1) THIS LINE GIVES WARNING WHEN LAUNCHED
    timeHelper = swarm.timeHelper

    num_cfs = len(swarm.allcfs.crazyflies)

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
        if FLY:
            swarm.allcfs.goToAbsolute(start_pos_list, duration=GOTO_DURATION)
            timeHelper.sleep(4.0)
            swarm.allcfs.land(targetHeight=0.05, duration=3.0)
            timeHelper.sleep(4.0)
        raise(e)

    except KeyboardInterrupt:
        print ("##### KeyboardInterrupt detected. Landing all CFs  #####")
        if FLY:
            swarm.allcfs.land(targetHeight=0.05, duration=3.0)
            timeHelper.sleep(4.0)

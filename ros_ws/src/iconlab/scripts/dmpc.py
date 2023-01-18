#!/usr/bin/env python
import casadi as cs
import numpy as np
from scipy.constants import g
from casadi import *
# from util import *
import logging
# import util

import argparse
import atexit
import csv
import datetime
import signal
import sys
import time
from time import perf_counter as pc

import dpilqr as dec
import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf
from dpilqr import plot_solve, split_agents
from distributed_mpc import *
from centralized_mpc import *
import pycrazyswarm as crazy



#################################################
theta_max = np.pi / 5
phi_max = np.pi / 5

v_max = 2
v_min = -2

theta_min = -np.pi / 5
phi_min = -np.pi / 5

tau_max = 15
tau_min = 0

x_min = -5
x_max = 5

y_min = -5
y_max = 5

z_min = 0
z_max = 4.0

a_max = 1.5
a_min = -1.5

# u_ref_base = np.array([0,0,g])
u_ref_base = np.array([0,0,0,0,0,0])

# max_input_base = np.array([[theta_max], [phi_max], [tau_max]])
# min_input_base = np.array([[theta_min], [phi_min], [tau_min]])
max_input_base = np.array([[v_max], [v_max], [v_max], [a_max],[a_max], [a_max]])
min_input_base = np.array([[v_min], [v_min], [v_min], [a_min],[a_min], [a_min]])

max_state_base = np.array([[x_max], [y_max], [z_max], [v_max],[v_max], [v_max]])
min_state_base = np.array([[x_min], [y_min], [z_min], [v_min],[v_min], [v_min]])

n_states = 6
n_inputs = 6
#############################################



plt.ion()

# Assemble filename for logged data
datetimeString = datetime.datetime.now().strftime("%m%d%y-%H:%M:%S")
csv_filename = "experiment_data/" + datetimeString + "-data.csv"

# Enable or disable data logging
LOG_DATA = False

TAKEOFF_Z = 1.0
TAKEOFF_DURATION = 3.0

# Used to tune aggresiveness of low-level controller
GOTO_DURATION = 2.5

# Defining takeoff and experiment start position

"""Case 1: 3 drones"""
start_pos_drone1 = [0.5, 1.0, 1]
start_pos_drone2 = [3.0, 2.5, 1]
start_pos_drone3 = [1.5, 1.0, 1]

goal_pos_drone1 = [2.5, 1.5, 1]
goal_pos_drone2 = [0.5, 1.5, 1]
goal_pos_drone3 = [1.5, 2.2, 1]

start_pos_list = [start_pos_drone1, start_pos_drone2, start_pos_drone3]
goal_pos_list = [goal_pos_drone1, goal_pos_drone2 , goal_pos_drone3]


"""Case 2: 5 drones"""
# start_pos_drones = paper_setup_5_quads()[0][dec.pos_mask([6]*5, 3)].flatten().tolist()
# goal_pos_drones = paper_setup_5_quads()[1][dec.pos_mask([6]*5, 3)].flatten().tolist()

# start_pos_drone1 = start_pos_drones[0:3]
# start_pos_drone2 = start_pos_drones[3:6]
# start_pos_drone3 = start_pos_drones[6:9]
# start_pos_drone4 = start_pos_drones[9:12]
# start_pos_drone5 = start_pos_drones[12:15]

# goal_pos_drone1 = goal_pos_drones[0:3]
# goal_pos_drone2 = goal_pos_drones[3:6]
# goal_pos_drone3 = goal_pos_drones[6:9]
# goal_pos_drone4 = goal_pos_drones[9:12]
# goal_pos_drone5 = goal_pos_drones[12:15]

# start_pos_list = [start_pos_drone1, start_pos_drone2, start_pos_drone3,
#                     start_pos_drone4,start_pos_drone5]
# goal_pos_list = [goal_pos_drone1, goal_pos_drone2 ,goal_pos_drone3, 
#                     goal_pos_drone4, goal_pos_drone5]


""" Case 3: 10 drones"""
# start_pos_drones = paper_setup_10_quads()[0][dec.pos_mask([6]*10, 3)].flatten().tolist()
# goal_pos_drones = paper_setup_10_quads()[1][dec.pos_mask([6]*10, 3)].flatten().tolist()

# for i in range(1,11):
#     f'start_pos_drone{0}.format(i)' = 

# start_pos_list = [start_pos_drone1, start_pos_drone2, start_pos_drone3,
#                     start_pos_drone4,start_pos_drone5]
# goal_pos_list = [goal_pos_drone1, goal_pos_drone2 ,goal_pos_drone3, 
#                     goal_pos_drone4, goal_pos_drone5]







def go_home_callback(swarm, timeHelper, start_pos_list):
    """Tell all quadcopters to go to their starting positions when program exits"""
    print("Program exit: telling quads to go home...")
    swarm.allcfs.goToAbsolute(start_pos_list,duration = GOTO_DURATION)
    timeHelper.sleep(3.0)
    swarm.allcfs.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(3.0)


"""
The states of the quadcopter are: px, py ,pz, vx, vy, vz
"""
def perform_experiment(centralized=False, sim=False):

    fig1 = plt.figure()
    fig2 = plt.figure()
    
    if not sim:
        # Wait for button press for take off
        input("##### Press Enter to Take Off #####")
    
        allcfs.takeoff(targetHeight=TAKEOFF_Z, duration=1.0+TAKEOFF_Z)
        timeHelper.sleep(TAKEOFF_DURATION)
        allcfs.goToAbsolute(start_pos_list)
    
        # Wait for button press to begin experiment
        input("##### Press Enter to Begin Experiment #####")

    n_agents = 3
    n_states = 6
    n_controls = 6
    n_dims = [3] * n_agents
    x_dims = [n_states] * n_agents

    Q = np.eye(n_states*n_agents) * 100
    R = np.eye(n_inputs*n_agents) * 0.01

    Qf = 1000.0 * np.eye(Q.shape[0])
    
    
    max_input = np.tile(max_input_base,n_agents)
    min_input = np.tile(min_input_base,n_agents)
    max_state = np.tile(max_state_base,n_agents)
    min_state = np.tile(min_state_base,n_agents)

    u_ref = np.tile(u_ref_base,n_agents)

    
    x = np.hstack([start_pos_list,np.zeros((n_agents,3))]).flatten() 
    x_goal = np.hstack([goal_pos_list,np.zeros((n_agents,3))]).flatten().reshape(-1,1)
    xi = x.reshape(-1, 1)

    dt = 0.1
    N = 15
    radius = 0.5
    
    ids = [100 + i for i in range(n_agents)]
    U = np.zeros((N, n_controls*n_agents))
   
    
    X_full = np.zeros((0, n_states*n_agents))
    U_full = np.zeros((0, n_controls*n_agents))
    X = np.tile(xi,(N+1, 1))
   
    n_humans = 0
    d_converge = 0.1

    loop = 0

    while not np.all(dec.distance_to_goal(xi,x_goal,n_agents,n_states,3) <= d_converge):
        t0 = pc()
        # How to feed state back into decentralization?
        #  1. Only decentralize at the current state.
        #  2. Offset the predicted trajectory by the current state.
        # Go with 2. and monitor the difference between the algorithm and VICON.
        if centralized:
            X, U, _ , J , failed_count, _ = solve_centralized(xi,x_goal,u_ref,
                               N,Q,R,Qf,n_agents,
                               n_states,n_inputs,
                               radius,max_input,
                               min_input,max_state,
                               min_state)
        else:
            
            X, U, _ , J , failed_count, _ = solve_distributed(
                        xi, x_goal, u_ref, N, n_agents, n_states, n_inputs, radius, ids,\
                        x_min,x_max,y_min,y_max,z_min,z_max,v_min,v_max,a_min,a_max,theta_max,\
                    theta_min,tau_max,tau_min,phi_max,phi_min,n_humans,n_dims)
        tf = pc()
        
        print(f"Solve time: {tf-t0}")

        # print(f'shape of X is {X.shape}')
        # print(f'X_full has shape {X_full.shape}')
        
        # Record which steps were taken for plotting.
        X_full = np.r_[X_full, X]
        # U_full = np.r_[U_full, U]
        print(f'Shape of X is {X.shape}')


        
        print(f'X_full has shape {X_full.shape} at the {loop}th loop')

        if failed_count !=0:
            print(f'Infeasibility detected! ')
            
            # xd = X_full[loop-1,:].reshape(n_agents, n_states)[:, :3]
            break
        
        loop +=1
        
        # x, y, z coordinates from the solved trajectory X.
        xd = X.reshape(n_agents, n_states)[:, :3]
        print(f'xd is {xd}')
        if not sim:
            swarm.allcfs.goToAbsolute(xd, duration = 1.5)

            pos_cfs = [cf.position() for cf in swarm.allcfs.crazyflies] #position update from VICON
            vel_cfs = [cf.velocity() for cf in swarm.allcfs.crazyflies] #velocity update from VICON
            # print(f'vel_cfs is {vel_cfs}')
            # print(f'pos_cfs is {pos_cfs}')
            # print(f'vel_cfs is {vel_cfs}')
            # x_prev = xi.copy()
            # x_prev = pos_cfs[dec.pos_mask(x_dims,3)]
            # dV = (pos_cfs - x_prev[0:3]) / dt
            # xi = np.hstack([pos_cfs, dV])
            xi = np.hstack([pos_cfs, vel_cfs]).flatten().reshape(-1,1)   
        
        else:
            xi = X.reshape(-1,1)
        # print(X.shape,xi.shape)
        # state_error = np.abs(X.reshape(-1,1) - xi)
        # print(f"CF states: \n{xi.reshape(n_agents, n_states)}\n")
        # print(f"Predicted state error: {state_error}")

        plt.figure(fig1.number)
        plt.clf()
        plot_solve(X_full, J, x_goal, x_dims, n_d=3)
        plt.title("Path Taken")
        plt.gca().set_zlim(0, 2)

        plt.figure(fig2.number)
        plt.clf()
        plot_solve(X, J, x_goal, x_dims, n_d=3)
        plt.title("Path Planned")
        plt.gca().set_zlim(0, 2)

        fig1.canvas.draw()
        fig2.canvas.draw()
        plt.pause(0.5)

        # # Replace the currently predicted states with the actual ones.
        # X[0, pos_mask(x_dims, 3)] = xi[pos_mask(x_dims, 3)]
        # # TODO: see if this velocity makes sense here.
        # X[0, ~pos_mask(x_dims, 3)] = xi[~pos_mask(x_dims, 3)]
        # X = np.tile(xi, (N+1,1))

        if LOG_DATA:
            timestampString = str(time.time())
            csvwriter.writerow([timestampString] + pos_cfs + vel_cfs)

        rate.sleep()

       
    if not sim:
        input("##### Press Enter to Go Back to Origin #####")
        swarm.allcfs.goToAbsolute(start_pos_list,duration = GOTO_DURATION*3)
        timeHelper.sleep(4.0)

        swarm.allcfs.land(targetHeight=0.05, duration=GOTO_DURATION)
        timeHelper.sleep(4.0)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--centralized", action="store_true", default=False)
    parser.add_argument("-s", "--sim", action="store_true", default=False)
    args = parser.parse_args()

    swarm = crazy.Crazyswarm()
    listener = tf.TransformListener()
    rate = rospy.Rate(4)

    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # TODO: succeed without collision avoidance.
    swarm.allcfs.setParam("colAv/enable", 0) 

    # Exit on CTRL+C. 
    signal.signal(signal.SIGINT, lambda *_: sys.exit(-1))

    # Tell the quads to go home when we're done.
    if not args.sim:
        atexit.register(go_home_callback, swarm, timeHelper, start_pos_list)


    if LOG_DATA:
        num_cfs = len(swarm.allcfs.crazyflies)
        print("### Logging data to file: " + csv_filename)
        csvfile = open(csv_filename, 'w')
        csvwriter = csv.writer(csvfile, delimiter=',')

        csvwriter.writerow(['# CFs', str(num_cfs)])
        csvwriter.writerow(["Timestamp [s]"] + num_cfs*["x_d", "y_d", "z_d", " x", "y", "z", "qw", "qx", "qy", "qz"])

    perform_experiment(args.centralized, args.sim)






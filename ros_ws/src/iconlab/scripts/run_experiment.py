#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from pycrazyswarm import *
import signal

import julia
jl = julia.Julia(compiled_modules=False)
from julia import Main


TAKEOFF_Z = 1.0
TAKEOFF_DURATION = 3.0

# Used to tune aggresiveness of low-level controller
GOTO_DURATION = 2.0

# Enable or disable using Julia MPC algorithm
USE_JULIA = True

# Loop frequency


def sigint_handler(sig, frame):
    """
    Catches the SIGINT signal generated when ctrl-c is pressed.
    Sends a land command to all CFs before exiting the program.
    """
    print("SIGINT received, landing all CFs")
    swarm.allcfs.land(0.05, TAKEOFF_DURATION, 0)

if __name__ == '__main__':
    #rospy.init_node('tf_listener')

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    listener = tf.TransformListener()

    rate = rospy.Rate(5.0)

    if USE_JULIA:
        print("### Import Julia Functions. May take a while...")

        Main.include("MPC-One-Quad_1.jl")
        LOOP_ITERS = Main.Julia_Functions.num_iters

        print("### Julia import complete.")
    else:
        LOOP_ITERS = 100

    # Wait for button press for take off
    raw_input("##### Press Enter to Take Off #####")

    # Set SIGINT signal handler to catch ctrl-c press
    signal.signal(signal.SIGINT, sigint_handler)

    cf.takeoff(TAKEOFF_Z, TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    timeHelper.sleep(2.0)
    cf.goTo([0.0, 1.0, 1.0], yaw=0.0, duration=2.0)

    # Wait for button press to begin experiment
    raw_input("##### Press Enter to Begin Experiment #####")

    for i in range(LOOP_ITERS):

        #try:
        #    (pos_cf1,rot_cf1) = listener.lookupTransform('/world', '/cf1', rospy.Time(0))
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    continue

        # Only reaches here if a /tf message was received
        #print("CF1 Position: " + str(pos_cf1))
        #print("CF1 Rotation: " + str(rot_cf1))

        #x_update = [pos_cf1[0], pos_cf1[1], pos_cf1[2], rot_cf1[3], rot_cf1[0], rot_cf1[1], rot_cf1[2]]
        #print("X Update: " + str(x_update))

        if USE_JULIA:
            #print(Main.Julia_Functions.t0)
            #print(Main.Julia_Functions.k_mpc)
            Main.Julia_Functions.t0 += Main.Julia_Functions.dt
            Main.Julia_Functions.k_mpc += 1

            Main.Julia_Functions.set_initial_time(Main.Julia_Functions.prob_mpc, Main.Julia_Functions.t0)

            xd = Main.Julia_Functions.discrete_dynamics(Main.Julia_Functions.integrate(Main.Julia_Functions.prob_mpc),
                                    Main.Julia_Functions.prob_mpc.model, Main.Julia_Functions.prob_mpc.Z[0])

            #xd = Main.Julia_Functions.states(Main.Julia_Functions.altro)[5]

            print("Desired Position: " + str(xd))

            cf.goTo(xd, yaw=0.0, duration=GOTO_DURATION)

            try:
                (pos_cf1,rot_cf1) = listener.lookupTransform('/world', '/cf1', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Only reaches here if a /tf message was received
            print("CF1 Position: " + str(pos_cf1))
            print("CF1 Rotation: " + str(rot_cf1))

            x_update = [pos_cf1[0], pos_cf1[1], pos_cf1[2], rot_cf1[3], rot_cf1[0], rot_cf1[1], rot_cf1[2]]
            print("X Update: " + str(x_update))


            Main.Julia_Functions.run_MPC_iterate(x_update,
                                                 Main.Julia_Functions.t0,
                                                 Main.Julia_Functions.k_mpc,
                                                 Main.Julia_Functions.prob_mpc,
                                                 Main.Julia_Functions.Z_track,
                                                 Main.Julia_Functions.X_traj,
                                                 Main.Julia_Functions.iters,
                                                 Main.Julia_Functions.times,
                                                 i+1)

            #x_d = Main.Julia_Functions.X_traj[i+2][0:3]
            #print(x_d)
            rate.sleep()


    raw_input("##### Press Enter to Go Back to Origin #####")

    cf.goTo([0.0, 0.0, 1.0], yaw=0.0, duration=3.0)
    timeHelper.sleep(1.0)

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

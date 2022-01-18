#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from pycrazyswarm import *

import julia
jl = julia.Julia(compiled_modules=False)
from julia import Main

#def callback():

Main.include("MPC-One-Quad_1.jl")

if __name__ == '__main__':
    #rospy.init_node('tf_listener')

    #listener = rospy.Subscriber("\tf", String, callback)
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    listener = tf.TransformListener()

    Z = 1.0
    TAKEOFF_DURATION = 3.0
    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    rate = rospy.Rate(10.0)

    for i in range(Main.Julia_Functions.num_iters):
    #for i in range(100):
        GOTO_DURATION = 3.0
        try:
            (trans,rot) = listener.lookupTransform('/cf1', '/world', rospy.Time(0))
            print(trans)
            print(rot)
            x_update = [trans[0], trans[1], trans[2], rot[0], rot[1],rot[2],rot[3]]
            print(x_update)
            #print(Main.Julia_Functions.t0)
            #print(Main.Julia_Functions.k_mpc)
            Main.Julia_Functions.run_MPC_iterate(x_update,Main.Julia_Functions.t0,Main.Julia_Functions.k_mpc,
           			Main.Julia_Functions.prob_mpc, Main.Julia_Functions.Z_track, Main.Julia_Functions.X_traj,Main.Julia_Functions.iters,Main.Julia_Functions.times,i+1)
            Main.Julia_Functions.t0 += Main.Julia_Functions.dt
            Main.Julia_Functions.k_mpc += 1

            x_d = Main.Julia_Functions.X_traj[i+1][0:3]
            print(x_d)


            cf.goTo(x_d, yaw=0.0, duration=GOTO_DURATION)
            #timeHelper.sleep(GOTO_DURATION + )


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #continue
            pass

        rate.sleep()

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

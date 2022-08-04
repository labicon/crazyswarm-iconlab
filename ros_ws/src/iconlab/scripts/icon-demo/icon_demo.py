#!/usr/bin/env python

"""Demonstration script to spell out ICON using 10 Crazyflies

NOTE: We assume you have pycrazyswarm on your PYTHONPATH. One way to do this
is by adding the following to your ~/.bash_aliases.

  alias crazy_init="export PYTHONPATH=$PYTHONPATH:/home/iconlab/iconlab/zjw4/crazyswarm/ros_ws/src/crazyswarm/scripts"

NOTE: this script is 90% copied from ``waypoints.py``.

"""

import numpy as np

import pycrazyswarm
from waypoints import Waypoint
from icon_traj import spell_icon, define_yaml_conf


N_AGENTS = 10

# Times for going between waypoints.
T_MOVE = 1.0
T_STAY = 2.0

# Configuration file path.
CF_YAML = "./UIUC.yaml"

# Trajectory path.
TRAJ_PATH = "./icon.csv"


def ingest_waypoints(traj):
    """Ingest some trajectories from csv and output into the Waypoint class
       (copied from waypoints.py)
    """

    waypoints = []
    lastAgent = None
    for row in traj:
        if lastAgent is None or lastAgent != row[0]:
            lastTime = 0.0
        waypoints.append(Waypoint(
            int(row[0]),
            row[1],
            row[2],
            row[3],
            row[4],
            row[4] - lastTime))
        lastTime = row[4]
        lastAgent = int(row[0])

    return waypoints


def main():

    # Create the trajectory file if necessary.
    spell_icon(TRAJ_PATH, W=1, H=1)

    # Load waypoints for all of the crazyflies.
    traj = np.loadtxt(TRAJ_PATH, delimiter=",")
    waypoints = ingest_waypoints(traj)

    # Define our configuration to line up with our trajectories (optional)
    pos_init = traj[:N_AGENTS, 1:4]
    pos_init[:,-1] = 0.0
    define_yaml_conf(CF_YAML, positions=pos_init.tolist())
    
    # Otherwise default to start on a line along the x-axis.
    # define_yaml_conf(CF_YAML)
    
    # Start up the swarm.
    swarm = pycrazyswarm.Crazyswarm(CF_YAML)
    time_helper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Execute the waypoints according to our csv.
    allcfs.takeoff(targetHeight=1.0, duration=T_STAY)
    last_time = 0.0
    for waypoint in waypoints:
        if not np.isclose(last_time, waypoint.arrival):
            time_helper.sleep(T_STAY)
        if waypoint.arrival == 0:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, T_MOVE)
        elif waypoint.duration > 0:
            time_helper.sleep(waypoint.arrival - last_time)
            last_time = waypoint.arrival
            pos = [waypoint.x, waypoint.y, waypoint.z]
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, T_MOVE)
    time_helper.sleep(T_STAY)
    
    # land
    allcfs.land(targetHeight=0.02, duration=T_STAY)
    time_helper.sleep(T_STAY)


if __name__ == "__main__":
    main()


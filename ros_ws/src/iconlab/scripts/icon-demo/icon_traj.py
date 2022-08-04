#!/usr/bin/env python

"""Generates the waypoints to spell out ICON with other random tools"""

from functools import reduce
import yaml

import numpy as np
from scipy.optimize import linear_sum_assignment


n_crazyflies = 10
n_dim = 3
Z_BASE = 1.0

def define_formations(n_crazyflies=10, W=3, H=4, Z_BASE=1):
    """Initialize formations as a dicitonary of np.ndarrays that spell ICON."""
    
    assert n_crazyflies >= 7
    
    theta = np.arange(0, 2*np.pi, 2*np.pi/n_crazyflies)
    theta_c = np.linspace(np.pi/2, 3*np.pi/2, n_crazyflies)

    formations = {
        'start': np.vstack([
            np.zeros(n_crazyflies),
            np.linspace(-W, W, n_crazyflies),
            np.zeros(n_crazyflies)
        ]),
        'I': np.vstack([
            [-W,  W, -W, W] + [0] * (n_crazyflies - 4),
            np.zeros(n_crazyflies),
            [-H, -H,  H, H] + np.linspace(-H, H, n_crazyflies-4).tolist()
        ]),
        'C': np.vstack([
            W * np.cos(theta_c).round(3) + 1.67,
            np.zeros(n_crazyflies),
            H * np.sin(theta_c).round(3)
        ]),
        'O': np.vstack([
            W * np.cos(theta).round(3),
            np.zeros(n_crazyflies),
            H * np.sin(theta).round(3)
        ]),
        'N': np.vstack([
            [-W, -W, W, W] + np.linspace(-W, W, n_crazyflies-4).tolist(),
            np.zeros(n_crazyflies),
            [-H, 0, 0, H] + np.linspace(H, -H, n_crazyflies-4).tolist()
        ])
    }

    # Raise the Z's to some height above the ground.
    lowest_z = reduce(lambda m, form: form[2].min(), formations.values())
    for formation in formations.values():
        formation[2] += Z_BASE - lowest_z
    
    return formations

    
def assign_drones(formations, order):
    """Assign drones to these base waypoints by minimizing the distance the 
       collective group has to move to the next waypoint. See here:
       https://en.wikipedia.org/wiki/Assignment_problem
    """

    base_waypoints = np.zeros((len(order), n_dim, n_crazyflies))
    base_waypoints[0] = formations[order[0]]

    # Compute base waypoints via linear sum assignment to minimize overlaps.
    for i in range(len(order)-1):
        # Find the best association to the next formation to minimize the sum of squared distances.
        distances = np.linalg.norm(
            np.moveaxis(base_waypoints[i, ..., np.newaxis], 0, 2) 
        - np.swapaxes(formations[order[i+1]][..., np.newaxis], 0, 2), 
        axis=2)
        start_ind, end_ind = linear_sum_assignment(distances)

        base_waypoints[i+1, :, start_ind] = formations[order[i+1]][:, end_ind].T

    return base_waypoints


def fill_waypoints(base_waypoints, order, trans_len=10, hold_len=5):
    """Add filler waypoints between the base waypoints"""
    
    wp_len = trans_len + hold_len
    waypoints = np.zeros((len(order) * wp_len, n_dim, n_crazyflies))
    waypoints[-wp_len:] = base_waypoints[-1]

    for i_wp in range(len(order)-1):
        for i_dim in range(n_dim):
            for i_agent in range(n_crazyflies):
                waypoints[i_wp*wp_len : i_wp*wp_len + trans_len, i_dim, i_agent] = np.linspace(
                    base_waypoints[i_wp, i_dim, i_agent], 
                    base_waypoints[i_wp+1, i_dim, i_agent],
                    trans_len)
                waypoints[i_wp*wp_len + trans_len : (i_wp+1)*wp_len, i_dim, i_agent] \
                    = base_waypoints[i_wp+1, i_dim, i_agent]

    return waypoints


def waypoints_to_csv(path, waypoints, T):
    """Flatten the waypoints from array into flat dictionary with assigned agents and timestamps

    Parameters
    ----------
    path : str
        Path to write the file to
    waypoints : np.ndarray
        Waypoints of shape (n_timesteps, n_dim, n_agents)
    T : float
        Amount of time to simulate over

    Returns
    -------
    dict

    """

    # Reorganize waypoints to be put into a csv by ensuring dimension gets 
    # reshaped the slowest.
    n_timesteps, n_dim, n_agents = waypoints.shape
    waypoints = waypoints.swapaxes(0, 1).reshape(n_dim, -1).T

    # Assign ID's and timestamps to each row.
    ids = np.tile(np.arange(n_agents), n_timesteps).reshape(-1,1)
    t = np.linspace(0.0, T, n_timesteps).repeat(n_agents).reshape(-1,1)
    
    trajectory = np.hstack([ids, waypoints, t])
    header = "id x[m] y[m] z[m] t[s]"
    np.savetxt(path, trajectory, header=header, fmt="%i, %f, %f, %f, %f")


def spell_icon(csvname="test.csv", *args, **kwargs):
    # Create the initial hardcoded formations.
    formations = define_formations(*args, **kwargs)

    # Determine which drones should go to which waypoints.
    order = ['start', 'I', 'C', 'O', 'N', 'start']
    waypoints = assign_drones(formations, order)

    # Output to csv file.
    T_PER_FORMATION = 5.0
    waypoints_to_csv(csvname, waypoints, T_PER_FORMATION * len(order))

    # Add some filler waypoints for more precise trajectories.
    # filled_waypoints = fill_waypoints(waypoints, order, trans_len=10, hold_len=3)
    # waypoints_to_csv(csvname, filled_waypoints, 20.0)


def define_yaml_conf(path, positions=None):
    """Fills out the crazyfile yaml file at the desired path
    
    Parameters
    ----------
    path : str
    positions : list
        List of tuples defining how all the flies should be laid out
        
    """

    # Default to 10 crazyflies linearly spaced across the x-axis.
    if positions is None:
        N = 10
        SPACING = 1.0
        positions = np.hstack([
            np.linspace(0, N*SPACING, N).reshape(-1,1).round(3) - N*SPACING/2.0,
            np.zeros((N, 2)),
        ]).tolist()

    cf_configs = [
        {
            'id': i,
            'initialPosition': position,
            'channel': 80,
            'type': "default"
        }
        for i, position in enumerate(positions)
    ]

    config = {'crazyflies': cf_configs}

    with open(path, 'w') as file:
        yaml.dump(config, file)


import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from sklearn.cluster import KMeans
import pandas as pd
from scipy.optimize import minimize
import time


def normalize(v):
    norm = np.linalg.norm(v, axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])


def curvature(waypoints: np.ndarray):
    """
    ##### TODO #####
    Curvature as  the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_waypoints] !!!!!
    """

    curvature = 0
    x = waypoints
    iter_num = waypoints.shape[1] - 1
    for n in np.linspace(1, iter_num-1, iter_num-1, dtype=int):
        curvature += unit_vector((x[:, n + 1] - x[:, n])).dot(unit_vector((x[:, n] - x[:, n - 1])))
    return curvature


def unit_vector(v):
    return v / np.linalg.norm(v)


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    """
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    """
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints) ** 2)

    # derive curvature
    curv = curvature(waypoints.reshape(2, -1))

    return -1 * weight_curvature * curv + ls_tocenter


def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=6, way_type="center"):
    """
    ##### TODO #####
    Predict waypoint via two different methods:
    - center
    - smooth 

    args:
        roadside1_spline
        roadside2_spline
        num_waypoints (default=6)
        parameter_bound_waypoints (default=1)
        waytype (default="smoothed")
    """
    # Create Spline
    t = np.linspace(0, 1, 6)
    lane_boundary1_points_points = np.array(splev(t, roadside1_spline))
    lane_boundary2_points_points = np.array(splev(t, roadside2_spline))
    # derive center between corresponding roadside points
    way_points_center = np.empty(shape=(2, num_waypoints))
    for i in range(2):
        for j in range(num_waypoints):
            way_points_center[i][j] = ((lane_boundary1_points_points[i][j] + lane_boundary2_points_points[i][j]) / 2)

    if way_type == "center":
        # output way_points with shape(2 x Num_waypoints)
        return way_points_center
    elif way_type == "smooth":
        ##### TODO #####

        # create spline arguments

        # derive roadside points from spline

        # derive center between corresponding roadside points (assumed)
        # smooth
        # optimization
        way_points_center = way_points_center.reshape((2*num_waypoints))
        way_points_center = minimize(smoothing_objective,
                              way_points_center,
                              args=way_points_center)["x"]

        return way_points_center.reshape(2, -1)


def target_speed_prediction(waypoints, num_waypoints_used=5,
                            max_speed=60, exp_constant=4.5, offset_speed=40):
    """
    Predict target speed given waypoints
    Implement the function using curvature()

    args:
        waypoints [2,num_waypoints]
        num_waypoints_used (default=5)
        max_speed (default=60)
        exp_constant (default=4.5)
        offset_speed (default=30)
    
    output:
        target_speed (float)
    """
    #if curvature(waypoints)<3.45:
    #    offset_speed = 35
    target_speed = (max_speed - offset_speed) * np.exp(-exp_constant * np.abs( \
        num_waypoints_used - 2 - curvature(waypoints))) + offset_speed
    print("curvature", curvature(waypoints))
    return target_speed

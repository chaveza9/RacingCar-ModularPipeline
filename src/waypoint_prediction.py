import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from sklearn.cluster import KMeans
import pandas as pd
from scipy.optimize import minimize
import time


def normalize(v):
    norm = np.linalg.norm(v,axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])

def curvature(waypoints):
    '''
    ##### TODO #####
    Curvature as  the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_waypoints] !!!!!
    '''

    return curvature


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    '''
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    '''
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints)**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2,-1))

    return -1 * weight_curvature * curv + ls_tocenter


def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=6, way_type = "smooth"):
    '''
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
    '''

    # derive center between corresponding roadside points (assumed)
    A = 155, 158, 182, 186, 195, 256, 263, 275, 284, 317, 317, 317, 316, 316, 312, 310, 305, 295, 296, 300, 294, 288, 280, 266, 251, 194, 189, 188, 187, 183, 142, 126, 101, 101, 101, 131, 189, 232, 290, 306, 309, 309, 309, 288, 266, 227, 218, 214, 208, 175, 156, 134, 89, 79, 72, 58, 57, 56, 69, 87, 132, 154, 167, 196, 234, 252, 260, 274, 281, 281, 281, 233, 201, 176, 132, 110, 90, 74, 74, 74, 75, 75, 77, 94, 108, 151
    B = 475, 475, 492, 492, 492, 519, 521, 524, 524, 524, 495, 494, 488, 487, 471, 471, 471, 484, 488, 492, 496, 499, 499, 499, 492, 467, 467, 467, 467, 467, 436, 420, 397, 387, 375, 360, 329, 284, 224, 178, 170, 163, 142, 132, 122, 122, 122, 124, 128, 133, 136, 136, 136, 126, 119, 124, 134, 146, 152, 161, 161, 161, 158, 151, 151, 151, 152, 153, 158, 165, 178, 248, 283, 308, 329, 340, 358, 374, 381, 382, 386, 388, 400, 421, 437, 475

    df = pd.DataFrame([list(A), list(B)]).T
    df.columns = ['A', 'B']


    if way_type == "center":
        ##### TODO #####
     
        # create spline arguments

        # derive roadside points from spline

        #center
        km = KMeans(n_clusters=20).fit(df)
        # pts = km.cluster_centers_[km.labels_]
        way_points = (km.cluster_centers_[:, 0], km.cluster_centers_[:, 1], '.')

        # output way_points with shape(2 x Num_waypoints)
        return way_points
    
    elif way_type == "smooth":
        ##### TODO #####

        # create spline arguments

        # derive roadside points from spline

        # derive center between corresponding roadside points (assumed)

        #smooth
        km = KMeans(n_clusters=12).fit(df)
        pts = km.cluster_centers_[km.labels_]
        way_points_center = (pts[:, 0], pts[:, 1])


        # optimization
        way_points = minimize(smoothing_objective, 
                      (way_points_center), 
                      args=way_points_center)["x"]

        return way_points.reshape(2,-1)


def target_speed_prediction(waypoints, num_waypoints_used=5,
                            max_speed=60, exp_constant=4.5, offset_speed=30):
    '''
    ##### TODO #####
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
    '''


    # return target_speed
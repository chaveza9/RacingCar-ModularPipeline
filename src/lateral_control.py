import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time


class LateralController:
    """
    Lateral control using the Stanley controller

    functions:
        stanley 

    init:
        gain_constant (default=5)
        damping_constant (default=0.5)
    """


    def __init__(self, gain_constant=0.6, damping_constant=0.05):
        self.gain_constant = gain_constant
        self.damping_constant = damping_constant
        self.previous_steering_angle = 0


    def stanley(self, waypoints, speed):
        """
        ##### TODO #####
        one step of the stanley controller with damping
        args:
            waypoints (np.array) [2, num_waypoints]
            speed (float)
        """
        # Extract constants
        k = self.gain_constant  # Steering Controller Gain
        epsilon = 1E-9          # Epsilon Constant to avoid division by 0
        delta = 0.1
        D = self.damping_constant   # Damping constant
        # derive orientation error as the angle of the first path segment to the car orientation
        dx = waypoints[0, 1] - waypoints[0, 0]
        dy = waypoints[1, 1] - waypoints[1, 0]
        psi =  np.arctan2(dx,dy)
        # derive cross track error as distance between desired waypoint at spline parameter equal zero ot the car position
        d = waypoints[0,0] - 96/2 ###### CHECK!!!!!!
        # derive stanley control law
        # prevent division by zero by adding as small epsilon
        steering_angle_cmd =  psi + np.arctan2(k*d,(speed+epsilon))
        print("steering angle:",steering_angle_cmd)
        #steering_angle_cmd = np.clip(steering_angle_cmd, -0.4, 0.4)
        # derive damping term
        damping = D*(steering_angle_cmd-self.previous_steering_angle)
        if steering_angle_cmd < 0:
            damping = np.clip(damping, steering_angle_cmd-delta*steering_angle_cmd, -steering_angle_cmd+delta*steering_angle_cmd )
        else:
            damping = np.clip(damping, -steering_angle_cmd-delta*steering_angle_cmd, steering_angle_cmd+delta*steering_angle_cmd)
        steering_angle = steering_angle_cmd - damping
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        steering_angle = np.clip(steering_angle, -0.4, 0.4) / 0.4
        # Save steering angle
        self.previous_steering_angle = steering_angle
        return steering_angle







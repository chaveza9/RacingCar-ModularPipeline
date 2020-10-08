import gym
from gym.envs.box2d.car_racing import CarRacing
from lane_detection import LaneDetection
from waypoint_prediction import waypoint_prediction, SpeedPrediction
from lateral_control import LateralController
from longitudinal_control import LongitudinalController
import matplotlib.pyplot as plt
import numpy as np
import pyglet
from pyglet import gl
from pyglet.window import key

# action variables
a = np.array( [0.0, 0.0, 0.0] )

# init environement
env = CarRacing()
env.render()
env.reset()

# define variables
total_reward = 0.0
steps = 0
restart = False

# init modules of the pipeline
LD_module = LaneDetection()
LatC_module = LateralController()
LongC_module = LongitudinalController()
Speed_module = SpeedPrediction(max_speed=65, exp_constant=8, offset_speed=40, num_waypoints_used=6)

# init extra plot
fig = plt.figure()
plt.ion()
plt.show()


while True:
    # perform step
    s, r, done, speed, info = env.step(a)

    # lane detection
    lane1, lane2 = LD_module.lane_detection(s)

    # waypoint and target_speed prediction
    waypoints = waypoint_prediction(lane1, lane2, way_type="smooth")
    target_speed = Speed_module.target_speed_prediction(waypoints)

    # control
    a[0] = LatC_module.stanley(waypoints, speed)
    a[1], a[2] = LongC_module.control(speed, target_speed)

    # reward
    total_reward += r

    # outputs during training
    if steps % 2 == 0 or done:
        print("\naction " + str(["{:+0.2f}".format(x) for x in a]))
        print("speed {:+0.2f} targetspeed {:+0.2f}".format(speed, target_speed))

        #LD_module.plot_state_lane(s, steps, fig, waypoints=waypoints, curv=curv)
        LongC_module.plot_speed(speed, target_speed, steps, fig)

    steps += 1
    env.render()

    # check if stop
    if done or restart or steps>=600: 
        print("step {} total_reward {:+0.2f}".format(steps, total_reward))
        break

env.close()
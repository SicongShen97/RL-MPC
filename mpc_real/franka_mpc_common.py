import numpy as np
import math
from matplotlib import pyplot as plt


def back_forth(a, s, v, t):
    # if v > 0:
    #     if s + v*dt <= a + center_x:
    #         return s + v*dt
    #     else:
    #         return a + center_x
    # elif v < 0:
    #     if s + v*dt >= center_x - a:
    #         return s + v*dt
    #     else:
    #         return center_x - a
    # else:
    #     return s
    if v == 0:
        return s
    p = 4*a / v
    return 2 * a / math.pi * math.asin(math.sin(s * math.pi / (2*a) + 2 * math.pi / p * t))


def move_obstacles(t, obstacles, vel, pos_dif, x_center):
    new_obstacles = []
    for i in range(obstacles.shape[0]):
        pre_rel_x_pos = obstacles[i][0] - x_center
        cur_rel_x_pos = back_forth(pos_dif, pre_rel_x_pos, vel, t)
        ob = obstacles[i].copy()
        ob[0] = cur_rel_x_pos + x_center
        new_obstacles.append(ob)
    return np.array(new_obstacles)


def extract_parameters(subgoal, goal, dt, N, obstacles, vel, pos_dif, x_center):
    goals = np.append(subgoal, goal)
    dyn_obs = obstacles.copy()

    return np.array([np.concatenate([goals, move_obstacles(dt*n, dyn_obs, vel, pos_dif, x_center).ravel()]) for n in range(N)])


def make_obs(p):
    obst = p[4:]
    n = obst.shape[0]
    obs = {'real_obstacle_info': np.array_split(obst, n / 4)}
    return obs


if __name__ == "__main__":
    goal = np.array([1.45, 0.43])
    dt = 0.1
    N = 10
    obstacles = np.array([[1.3, 0.60, 0.03, 0.03]])
    vel = -0.02
    pos_dif = 0.22
    x_center = 1.3
    parameters = extract_parameters(goal, goal, dt, N, obstacles, vel, pos_dif, x_center)
    print(parameters)
    plt.plot(range(N), parameters[:, 4])
    plt.show()

import numpy as np
import math
from matplotlib import pyplot as plt


def back_forth(a, s, v, t):
    if v == 0:
        return s
    p = 4*a / v
    return 2 * a / math.pi * math.asin(math.sin(s * math.pi / (2*a) + 2 * math.pi / p * t))


def move_obstacles(t, obstacles, vels, pos_dif, x_center):
    new_obstacles = []
    for i in range(obstacles.shape[0]):
        pre_rel_x_pos = obstacles[i][0] - x_center
        cur_rel_x_pos = back_forth(pos_dif, pre_rel_x_pos, vels[i], t)
        ob = obstacles[i].copy()
        ob[0] = cur_rel_x_pos + x_center
        new_obstacles.append(ob)
    return np.array(new_obstacles)


def extract_parameters(subgoal, goal, dt, N, obstacles, vels, pos_dif, x_center):
    goals = np.append(subgoal, goal)
    dyn_obs = obstacles.copy()

    return np.array([np.concatenate([goals, move_obstacles(dt*n, dyn_obs, vels, pos_dif, x_center).ravel()]) for n in range(N)])


def make_obs(p):
    obst = p[6:]
    n = obst.shape[0]
    obs = {'real_obstacle_info': np.array_split(obst, n / 6)}
    return obs


if __name__ == "__main__":
    goal = np.array([0.5 + 0.8, -0.3 + 0.75, 0.4])
    dt = 0.1
    N = 10
    dyn_obstacles = np.array([[0 + 0.5 + 0.8, 0.1 + 0.75, 0.4, 0.015, 0.017, 0.015],
                              [0 + 0.5 + 0.8, -0.1 + 0.75, 0.4, 0.015, 0.017, 0.015]])
    vels = [0.0, 0.0]
    pos_dif = 0.1
    x_center = 1.3
    parameters = extract_parameters(goal, goal, dt, N, dyn_obstacles, vels, pos_dif, x_center)
    print(parameters)
    print(np.array(make_obs(parameters[0])["real_obstacle_info"]))
    # plt.plot(range(N), parameters[:, 4])
    # plt.show()

import argparse
import numpy as np
import math

def triangle_wave(a, s, p, t):
    return 2 * a / math.pi * math.asin(math.sin(s * 2 * math.pi + 2 * math.pi / p * t))


def move_obstacles(t, obstacles, vels, pos_difs, shifts):
    new_obstacles = []
    for i in range(len(obstacles)):
        rel_x = _compute_obstacle_rel_x_positions(t, i, pos_difs, vels, shifts)
        ob = obstacles[i].copy()
        ob[0] = ob[0] + rel_x
        new_obstacles.append(ob)

    return np.array(new_obstacles)


def _compute_obstacle_rel_x_positions(t, i, pos_difs, vels, shifts):
    max_q = pos_difs[i]
    s_q = max_q * 2
    v = vels[i]
    a = max_q  # amplitude
    p = s_q / v  # period
    s = shifts[i]  # 0.1  # time shift
    new_pos_x = triangle_wave(a, s, p, t)  # triangle wave
    return new_pos_x


def extract_parameters(subgoal, goal, t, dt, N, obstacles, vels, shifts, pos_difs, stat_obstacles):
    goals = np.append(subgoal, goal)
    stat_obstacles_np = np.array(stat_obstacles).ravel()
    return np.array([np.concatenate([goals,
                                     move_obstacles(t + dt * n, obstacles, vels, pos_difs, shifts).ravel(),
                                     stat_obstacles_np]) for n in range(N)])


def make_obs(p):
    obst = p[6:]
    n = obst.shape[0]
    obs = {'real_obstacle_info': np.array_split(obst, n / 6)}
    return obs


def str2bool(value):
    if isinstance(value, bool):
        return value
    if value.lower() in ['yes', 'true', 't', 'y', '1']:
        return True
    elif value.lower() in ['no', 'false', 'f', 'n', '0']:
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def get_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--mpc_gen', help='Generate MPC', type=str2bool, default=True)
    args = parser.parse_args()

    return args
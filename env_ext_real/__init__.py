import gym
import env_ext_real.franka as franka_env
from .utils import goal_distance, goal_distance_obs

Robotics_envs_id = [
    'FrankaPickDynSqrObstacle-v1',
    'FrankaPickDynSqrObstacles-v1',
    'FrankaPickDynObstacles-v1',
    'FrankaPickDynObstacles-v2'
]


def make_env(args):
    assert args.env in Robotics_envs_id
    if args.env[:6] == 'Franka':
        return franka_env.make_env(args)
    else:
        return None


def clip_return_range(args):
    gamma_sum_min = args.reward_min / (1.0 - args.gamma)
    gamma_sum_max = args.reward_max / (1.0 - args.gamma)
    return {
        'FrankaPickDynSqrObstacle-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaPickDynSqrObstacles-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaPickDynObstacles-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaPickDynObstacles-v2': (gamma_sum_min, gamma_sum_max),
    }[args.env]

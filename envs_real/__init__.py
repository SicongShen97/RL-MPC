import gym
from .franka.franka_pick_dyn_sqr_obstacle import FrankaPickDynSqrObstacleEnv

def register_custom_envs():
    gym.envs.register(
        id='FrankaPickDynSqrObstacle-v1',
        entry_point='envs_real:FrankaPickDynSqrObstacleEnv',
        max_episode_steps=100,
        kwargs={'reward_type': 'sparse', 'n_substeps': 20},
    )
    gym.envs.register(
        id='FrankaPickDynSqrObstacles-v1',
        entry_point='envs_real:FrankaPickDynSqrObstaclesEnv',
        max_episode_steps=100,
        kwargs={'reward_type': 'sparse', 'n_substeps': 20},
    )
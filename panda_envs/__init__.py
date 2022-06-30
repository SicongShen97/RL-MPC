import gym
from .fetch.panda_pick_dyn_labyrinth import PandaFetchPickDynLabyrinthEnv

def register_panda_env():
    gym.envs.register(
        id='PandaFetchPickDynLabyrinth-v1',
        entry_point='panda_envs:PandaFetchPickDynLabyrinthEnv',
        max_episode_steps=100,
        kwargs={'reward_type': 'sparse', 'n_substeps': 20},
    )
import time
from typing import List
import numpy as np
import os

from env_ext_real.franka import MPCControlGoalEnv
from policies_real import RLPolicy, MPCPolicy, Policy


class MPCRLPolicy(Policy):
    Vector = List[np.ndarray]
    InfoVector = List[dict]

    def __init__(self, args):
        # get current policy from path (restore tf session + graph)
        self.mpc_policy = MPCPolicy(args)
        self.rl_policy = RLPolicy(args)

    # def reset(self):
    #     self.rl_policy.reset()
    #     self.mpc_policy.reset()

    @property
    def model(self):
        return self.mpc_policy.model

    def set_env(self, env):
        super().set_env(env)
        env.disable_action_limit()

    # predicts next actions for given states (observations)
    def predict(self, obs):
        rl_actions = self.rl_policy.predict(obs)
        sub_goal = self.env.subgoal(rl_actions[0], obs[0])
        self.mpc_policy.set_sub_goal(sub_goal)

        actions = self.mpc_policy.predict(obs)

        return actions




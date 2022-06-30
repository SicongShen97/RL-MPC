from typing import List
import numpy as np
import os

from env_ext.fetch import MPCControlGoalEnv
from policies import RLPolicy
from policies.mpc_policy import MPCPolicy
from policies.policy import Policy


class MPCRLPolicy(Policy):
    Vector = List[np.ndarray]
    InfoVector = List[dict]

    def __init__(self, args):
        # get current policy from path (restore tf session + graph)
        self.mpc_policy = MPCPolicy(args)
        self.rl_policy = RLPolicy(args)

    def reset(self):
        self.rl_policy.reset()
        self.mpc_policy.reset()

    @property
    def model(self):
        return self.mpc_policy.model

    def set_envs(self, envs: List[MPCControlGoalEnv]):
        # environments references for the MPC
        self.envs = envs
        self.mpc_policy.set_envs(envs)

        for env in envs:
            env.disable_action_limit()

    # predicts next actions for given states (observations)
    def predict(self, obs: Vector) -> (Vector, InfoVector):
        actions, infos = self._my_step_batch(obs)
        return actions, infos

    def initial_info(self, obs: Vector) -> InfoVector:
        return self.mpc_policy.initial_info(obs)

    def _my_step_batch(self, obs):
        # compute actions from obs based on current policy by running tf session initialized before
        rl_actions, rl_infos = self.rl_policy.predict(obs)

        sub_goals = [env.subgoal(rl_action) for (env, rl_action) in zip(self.envs, rl_actions)]
        self.mpc_policy.set_sub_goals(sub_goals)

        actions, infos = self.mpc_policy.predict(obs)

        # gripper action
        for i in range(len(actions)):
            actions[i][3] = rl_actions[i][3]

        return actions, infos

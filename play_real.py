import numpy as np
import os
from env_ext_real import make_env
from algorithm.replay_buffer import goal_based_process
from mpc_real import MPCDebugPlot
from policies_real import MPCPolicy, RLPolicy, Policy, make_policy
from common_real import get_args
from gym.wrappers.monitoring.video_recorder import VideoRecorder
from envs_real import register_custom_envs
import sys


class Player:
    policy: Policy = None
    debug_plot: MPCDebugPlot = None
    # debug_plot: MPCDebugPlotSmall = None
    mpc_policy = False

    def __init__(self, args):
        # initialize environment
        self.args = args
        self.env = make_env(args)
        # self.args.timesteps = self.env.env.env.spec.max_episode_steps
        self.info = []
        self.test_rollouts = 1
        self.mode = args.play_mode
        self.policy = make_policy(args)
        self.policy.set_envs(envs=[self.env])
        if args.play_policy in ['MPCRLPolicy', 'MPCPolicy']:
            self.mpc_policy = True
            if self.mode == 'plot':
                self.sim_length = args.timesteps
                self.debug_plot = MPCDebugPlot(args, sim_length=self.sim_length, model=self.policy.model)
                #self.debug_plot = MPCDebugPlotSmall(args, sim_length=self.sim_length, model=self.policy.model)

    def play(self):
        # play policy on env
        pass


if __name__ == "__main__":
    register_custom_envs()
    # Call play.py in order to see current policy progress
    args = get_args()
    player = Player(args)
    player.play()
    # player.record_video(raw_path="/home/ssc/bachelor-thesis/videos/rollouts_{}_{}".format(args.env, args.play_policy))

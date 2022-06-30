import numpy as np
import os
from env_ext import make_env
from algorithm.replay_buffer import goal_based_process
from mpc import MPCDebugPlot
from policies import MPCPolicy, RLPolicy, Policy, make_policy
from common import get_args
from gym.wrappers.monitoring.video_recorder import VideoRecorder
from envs import register_custom_envs
import sys


class Player:
    policy: Policy = None
    debug_plot: MPCDebugPlot = None
    #debug_plot: MPCDebugPlotSmall = None
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
        env = self.env
        acc_sum, obs = 0.0, []
        err_sum = 0
        #seed = 5995
        #seed = 1240
        # seed = 4014
        seed = 3019
        env.np_random.seed(seed)  # TODO remove

        for i in range(self.test_rollouts):
            env.np_random.seed(seed + i)
            print('Seed: ', seed + i)
            self.policy.reset()
            ob = env.reset()
            obs.append(goal_based_process(ob))

            if args.play_policy in ['MPCPolicy']:
                self.policy.set_sub_goals([ob['desired_goal']])

            if self.debug_plot:
                info = self.policy.initial_info([ob])[0]
                self.debug_plot.createPlot(xinit=info['xinit'], pred_x=info['pred_x'],
                                           pred_u=info['pred_u'], k=0, parameters=info['parameters'], obs=ob)

            prev_ob = ob
            sim_timestep = 0
            for timestep in range(args.timesteps):
                actions, infos = self.policy.predict(obs=[ob])
                action = actions[0]
                ob, _, _, env_info = env.step(action)

                if self.mpc_policy:
                    info = infos[0]
                    # sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n" \
                    #                  .format(info['info'].it, info['info'].solvetime))
                    mpc_info = info
                    next_x = mpc_info['next_x']
                    target_x = mpc_info['target_x']
                    pred_x = mpc_info['pred_x']
                    pred_u = mpc_info['pred_u']
                    parameters = mpc_info['parameters']

                    if self.debug_plot:
                        print('t:', sim_timestep)
                        self.debug_plot.updatePlots(next_x=next_x, target_x=target_x, pred_x=pred_x, pred_u=pred_u, k=sim_timestep,
                                                parameters=parameters, ob=prev_ob, new_ob=ob)
                    sim_timestep = sim_timestep + 1

                    #print('Reward: ', env_info['Rewards'])
                    #print('MPC flag: ', mpc_info['exitflag'])

                    if mpc_info['exitflag'] != 1:
                        print("FORCESPRO error")
                        err_sum += 1

                    if ob['collision_check']:
                        sys.stderr.write("Collision on {}. Exiting.\n" \
                                         .format(ob['observation'][0:2]))
                        if self.debug_plot:
                            self.debug_plot.show()
                        else:
                            env.render()
                        break

                    if env_info['Success']:
                        print('Success. Exiting.')
                        if self.debug_plot:
                            self.debug_plot.show()
                        else:
                            env.render()
                        break

                    if self.debug_plot:
                        if timestep == args.timesteps - 1:
                            self.debug_plot.show()
                        else:
                            self.debug_plot.draw()
                            #input('check')
                    else:
                        env.render()
                else:
                    env.render()
                    if env_info['Success']:
                        print('Success 2. Exiting.')
                        break
                prev_ob = ob
            print('Collisions: ', env.collisions)
            print('Errors: ', err_sum)

    def record_video(self, raw_path="myrecord"):
        env = self.env
        test_col_tolerance = 0
        test_rollouts = 30
        seed = 3000
        env.np_random.seed(seed)  # TODO remove
        # play policy on env
        recorder = VideoRecorder(env.env.env, base_path=raw_path)
        recorder.frames_per_sec = 30
        acc_sum, obs = 0.0, []
        tol_acc_sum = 0.0
        for i in range(test_rollouts):
            env.np_random.seed(seed + i)
            print('Seed: ', seed + i)
            self.policy.reset()
            ob = env.reset()
            env_info = None
            print("Rollout {}/{} ...".format(i + 1, test_rollouts))
            for timestep in range(self.args.timesteps):
                actions, infos = self.policy.predict(obs=[ob])
                action = actions[0]
                ob, _, _, env_info = env.step(action)
                recorder.capture_frame()
                if env_info['Success']:
                    print('Success. Exiting.')
                    break
            acc_sum += env_info['Success']
            if env.collisions <= test_col_tolerance:
                tol_acc_sum += env_info['Success']
            print("... done.")
            print('Collisions: ', env.collisions)
        print('Success rate: {}'.format(acc_sum / test_rollouts))
        print('Success rate (no col): {}'.format(tol_acc_sum / test_rollouts))
        recorder.close()


if __name__ == "__main__":
    register_custom_envs()
    # Call play.py in order to see current policy progress
    args = get_args()
    player = Player(args)
    player.record_video(raw_path="/home/ssc/bachelor-thesis/videos/rollouts_{}_{}".format(args.env, args.play_policy))

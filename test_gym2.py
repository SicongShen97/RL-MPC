import argparse
import sys

import numpy as np
import gym
import time
#import gym.envs.robotics.fetch_env
import gym.envs.robotics.fetch.push
from env_ext import make_env, clip_return_range, Robotics_envs_id
from envs import register_custom_envs


def get_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--env', help='gym env id', type=str, default='FetchReach-v1', choices=Robotics_envs_id)
    parser.add_argument('--goal', help='method of goal generation', type=str, default='vanilla',
                        choices=['vanilla', 'fixobj', 'interval', 'mpc', 'col_avoidance', 'ghgg_custom'])
    parser.add_argument('--play_policy', help='Policy to choose an action', type=str, default='RLPolicy',
                        choices=['MPCRLPolicy', 'MPCPolicy', 'RLPolicy'])
    parser.add_argument('--env_n_substeps', help='Steps to simulate', type=np.int32, default=20)

    args = parser.parse_args()

    return args

register_custom_envs()
args = get_args()

env = make_env(args)
ob = env.reset()
i = 0
pos_old = ob['observation'][0:3]
no_action = np.array([0,0,0,0])
go_z = np.array([0,0,1.00,-0.8])

close_action = np.array([0,0,0,0.3])

#print("vels: ", env.env.env.current_obstacle_vels)

while i < 100:
    env.np_random.seed(200)
    ob = env.reset()

    # action = close_action  # env.action_space.sample()
    # ob, reward, _, info = env.step(action)
    # env.render()
    print('space: ', env.action_space)

    print('Rollout: ', i)
    print('Time: ', env.time)
    #for k in range(140):
        # env.render()
        # time.sleep(0.05)
    for k in range(40):
        action = go_z #env.action_space.sample()
        #action = env.action_space.sample()
        ob, reward, _, info = env.step(action)
        env.render()
        #print(ob['observation'])
        #print(ob['observation'], ob['real_obstacle_info'])
        print('Reward: ', reward)
        #print(info)
        if ob['collision_check']:
             sys.stderr.write("Collision on {}. Exiting.\n" \
                              .format(ob['observation'][0:2]))

    #time.sleep(2)

    #print(ob['real_obstacle_info'])
    #print(ob['object_dis'])
    #print(ob['object_pos'])
    #print(ob['observation'])
    #print(ob['obj_vels'])
    #time.sleep(1)
    i = i + 1
from envs_real import register_custom_envs
from env_ext_real import make_env
from common_real import get_args
from policies import make_policy
import numpy as np

register_custom_envs()
args = get_args()
args.env = "FrankaPickDynLiftedObstacles-v1"
# args.env_n_substeps = 100
# args.play_path = "log/simple_net-ddpg2-FrankaPickDynLiftedObstacles-v1-hgg"
# args.play_epoch = 'best'
# args.play_policy = "RLPolicy"
env = make_env(args)
import time
# env = FrankaFetchPickDynSqrObstacleEnv()
# policy = make_policy(args)
# policy.set_envs(envs=[env])
obs = env.reset()
env.env.env.env.goal[0] = 0.0 + 0.5 + 0.8
# print(obs['achieved_goal'])
print(obs["observation"])
# exit()
# action = [0, -0.5, 0., 0]
action = [0, -0, 0, 0]
pre_obs = None
pre_grip = None
t1 = time.time()
for i in range(1000):
    # print(env.env)
    obs['desired_goal'][0] = 0.0 + 0.5 + 0.8
    achieved = obs['observation'][:3]
    desired = obs['desired_goal'][:3]
    # print("obs['achieved_goal']", obs['desired_goal'])
    # obs = env.reset()
    # exit()
    # print("obs['observation'][:3]", achieved)
    print('dist', np.linalg.norm(obs['achieved_goal'] - desired, axis=-1))
    # print(obs['observation'])
    # actions, infos = policy.predict(obs=[obs])
    # print(actions)
    # action = actions[0]
    action = [0, -0.5, 1, 0.]
    # action[0] = (-1)**i*0.7
    # print("action:", action)
    action[0] = 0.5*(-1)**i
    # id = env.sim.model.geom_name2id("obstacle:geom")  # id 36

    # for i in range(env.sim.data.ncon):
    #     contact = env.sim.data.contact[i]
    #     if (contact.geom1 == id) or (contact.geom2 == id):
    #         print("collide")
    #         break
    obs, reward, done, info = env.step(action)
    # print(obs["observation"])
    # print(obs['desired_goal'])
    # exit()
    # print(obs['object_dis'])
    if info['Success'] == 1.0:
        print(i)
        break
    print(info['Collisions'])
    env.render()
# print(obs["observation"])
    # time.sleep(1)
    # print(info)
    # print(obs["observation"])
    # print("obs['object_dis']", obs['object_dis'])
    # if info['Success'] == 1.0:
    #     print(i)
    #     break
    # print(obs["observation"][-3:])
# print(time.time() - t1)
    # obs_pos = env.sim.data.get_site_xpos('o')
    # grip_pos = obs["observation"][:3]
    # if pre_obs is not None:
        # print("calculated vel:", (obs_pos - pre_obs) / env.dt)
        # print("calculated vel of grip:", (grip_pos - pre_grip) / env.dt)
    # id = env.sim.model.geom_name2id("obstacle:geom") #  id 36
    # for i in range(env.sim.data.ncon):
    #     contact = env.sim.data.contact[i]
    #     if (contact.geom1 == id) or (contact.geom2 == id):
    #         print("collide")
    #         break
    # print(obs["observation"])
    # print(obs['real_obstacle_info'])
    # pre_obs = obs_pos.copy()
    # pre_grip = grip_pos.copy()
    # print("read vel:", env.sim.data.get_site_xvelp('o'))



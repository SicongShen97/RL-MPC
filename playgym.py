from envs import register_custom_envs
from env_ext import make_env
from common import get_args
from policies import make_policy
#
register_custom_envs()
args = get_args()
args.env = "FetchPickDynLiftedObstaclesEnv-v1"
# args.env_n_substeps = 100
# args.play_path = "log_real/simple_net-ddpg2-FrankaPickDynObstacles-v2-hgg"
# args.play_epoch = '19'
# args.play_policy = "RLPolicy"
env = make_env(args)
import time
# env = FrankaFetchPickDynSqrObstacleEnv()
# policy = make_policy(args)
# policy.set_envs(envs=[env])
obs = env.reset()
# env.env.env.env.goal[0] = 0.07 + 0.5 + 0.8
# print(obs['achieved_goal'])
# print(obs["observation"])

# action = [0, -0.5, 0., 0]
# action = [0, -0, 0, 0]
pre_obs = None
pre_grip = None
t1 = time.time()
for i in range(1000):
    # print(env.env)
    obs['desired_goal'][0] = 0.07 + 0.5 + 0.8
    print(obs['object_dis'])
    # actions, infos = policy.predict(obs=[obs])
    # print(actions)
    # action = actions[0]
    action = [0, -0.2, 0.2, -1]
    # print("action:", action)
    # action[0] = 1*(-1)**i
    # id = env.sim.model.geom_name2id("obstacle:geom")  # id 36

    # for i in range(env.sim.data.ncon):
    #     contact = env.sim.data.contact[i]
    #     if (contact.geom1 == id) or (contact.geom2 == id):
    #         print("collide")
    #         break
    obs, reward, done, info = env.step(action)
    # print(obs['object_dis'])
    # if info['Success'] == 1.0:
    #     print(i)
    #     break
    # print(info['Collisions'])
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



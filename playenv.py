from envs_real import register_custom_envs
from env_ext_real import make_env
from common import get_args
#
register_custom_envs()
args = get_args()
args.env = "FrankaPickDynSqrObstacle-v1"
env = make_env(args)
# env.disable_action_limit()
# env.reset()
# pre = None
# pre_grip = None
# for i in range(250):
#     env.render()
#     action = [0, 0.01, 0, 0]
#     obs, reward, done, info = env.step(action)
#     grip_pos = env.sim.data.get_site_xpos('grip_site')
#     if pre is not None:
#         print("delta:", obs["observation"][:3] - pre)
#         print("calculated vel:", (obs["observation"][:3] - pre) / env.dt)
#         print("delta of grip site:", grip_pos - pre_grip)
#         print("obs vel:", env.sim.data.get_site_xvelp('o'))
#         # print("grip_pos:", grip_pos)
#         # print("pre_grip:", pre_grip)
#     pre_grip = grip_pos.copy()
#     pre = obs["observation"][:3]
#     print("grip vel:", env.sim.data.get_site_xvelp('grip_site'))
#     # print("obs_vel:", )
#     print("-"*20)

# from envs_real.franka.franka_pick_dyn_sqr_obstacle import FrankaFetchPickDynSqrObstacleEnv
# from envs.fetch.franka_pick_dyn_sqr_obstacle import FrankaFetchPickDynSqrObstacleEnv
import time
# env = FrankaFetchPickDynSqrObstacleEnv()
env.reset()
action = [0, 0, 1, 0]
pre_obs = None
pre_grip = None
for _ in range(1):
    print(env.env)
    # env.render()
    # obs, reward, done, info = env.step(action)
    # time.sleep(1)
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



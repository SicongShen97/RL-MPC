from policies_real import make_policy
import numpy as np
from envs_real import register_custom_envs
from env_ext_real import make_env
from common_real import get_args
args = get_args()
register_custom_envs()
# policy = MPCPolicy(args)
# policy.initialize()
# policy.play()
env = make_env(args)
obs = env.reset()
print(obs)
action = np.array([0, -0.1, 0, 0])
for _ in range(10):
    obs, _, _, _ = env.step(action)
    print("*"*20)
    print(obs)



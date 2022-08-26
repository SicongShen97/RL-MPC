from policies_real.franka_mpc_policy import MPCPolicy
from common_real import get_args
args = get_args()
policy = MPCPolicy(args)
policy.initialize()
policy.play()
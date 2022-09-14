from .franka_policy import Policy
from .franka_rl_policy import RLPolicy
from .franka_mpc_policy import MPCPolicy
from .franka_mpc_rl_policy import MPCRLPolicy


def make_policy(args):
    return {
        'MPCRLPolicy': MPCRLPolicy,
        'MPCPolicy': MPCPolicy,
        'RLPolicy': RLPolicy,
    }[args.play_policy](args)

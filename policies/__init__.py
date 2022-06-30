from .policy import Policy
from .rl_policy import RLPolicy
from .mpc_policy import MPCPolicy
from .mpc_rl_policy import MPCRLPolicy

def make_policy(args):
    return {
        'MPCRLPolicy': MPCRLPolicy,
        'MPCPolicy': MPCPolicy,
        'RLPolicy': RLPolicy,
    }[args.play_policy](args)

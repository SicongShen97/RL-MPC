import forcespro
from .franka_plot import MPCDebugPlot
from .franka_pick_dyn_obstacle import generate_pathplanner
from .franka_mpc_common import extract_parameters, make_obs, get_args

def make_mpc(args):

    a = {
        'FrankaPickDynSqrObstacle-v1': franka_pick_dyn_obstacle,
    }

    return a[args.env].generate_pathplanner(create=args.mpc_gen, path=args.mpc_path)
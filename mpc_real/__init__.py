import forcespro
from .franka_plot import MPCDebugPlot
from .franka_pick_dyn_sqr_obstacles import generate_pathplanner
from .franka_pick_dyn_obstacles import generate_pathplanner
from .franka_mpc_common import extract_parameters, make_obs


def make_mpc(args):
    a = {
        'FrankaPickDynSqrObstacles-v1': franka_pick_dyn_sqr_obstacles,
        'FrankaPickDynObstacles-v1': franka_pick_dyn_obstacles,
        'FrankaPickDynObstacles-v2': franka_pick_dyn_obstacles
    }

    return a[args.env].generate_pathplanner(create=args.mpc_gen, path=args.mpc_path)
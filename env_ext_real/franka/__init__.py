from .mpc_control import MPCControlGoalEnv

def make_env(args):
	return {
		'mpc': MPCControlGoalEnv,
	}[args.goal](args)
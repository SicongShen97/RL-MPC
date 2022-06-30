from .ddpg import DDPG
from .ddpg2 import DDPG

def create_agent(args):
	return {
		'ddpg': ddpg.DDPG,
		'ddpg2': ddpg2.DDPG
	}[args.alg](args)
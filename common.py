import numpy as np
import copy
from env_ext import make_env, clip_return_range, Robotics_envs_id
from utils2.os_utils import get_arg_parser, get_logger, str2bool
from algorithm import create_agent
from learner import create_learner, learner_collection
from test import Tester
from algorithm.replay_buffer import ReplayBuffer_Episodic, goal_based_process

def get_args():
	parser = get_arg_parser()

	parser.add_argument('--tag', help='terminal tag in logger', type=str, default='')
	parser.add_argument('--alg', help='backend algorithm', type=str, default='ddpg', choices=['ddpg', 'ddpg2'])
	parser.add_argument('--learn', help='type of training method', type=str, default='hgg', choices=learner_collection.keys())

	parser.add_argument('--env', help='gym env id', type=str, default='FetchReach-v1', choices=Robotics_envs_id)
	args, _ = parser.parse_known_args()

	parser.add_argument('--goal', help='method of goal generation', type=str, default='vanilla',
						choices=['vanilla', 'fixobj', 'interval', 'mpc', 'col_test', 'ghgg_custom'])
	if args.env[:5] == 'Fetch':
		parser.add_argument('--init_offset', help='initial offset in fetch environments', type=np.float32, default=1.0)

	parser.add_argument('--play_path', help='path to meta_file directory for play', type=str, default=None)
	parser.add_argument('--play_epoch', help='epoch to play', type=str, default='latest')
	parser.add_argument('--play_mode', help='render mode', type=str, default='mujoco', choices=['mujoco', 'plot'])

	parser.add_argument('--gamma', help='discount factor', type=np.float32, default=0.98)
	parser.add_argument('--clip_return', help='whether to clip return value', type=str2bool, default=True)
	# these two arguments might be helpful if using other than sparse reward (-1, 0)
	parser.add_argument('--reward_min', help='discount factor', type=np.float32, default=-1.)
	parser.add_argument('--reward_max', help='discount factor', type=np.float32, default=0.)
	parser.add_argument('--eps_act', help='percentage of epsilon greedy explorarion', type=np.float32, default=0.3)
	parser.add_argument('--std_act', help='standard deviation of uncorrelated gaussian explorarion', type=np.float32, default=0.2)

	parser.add_argument('--pi_lr', help='learning rate of policy network', type=np.float32, default=1e-3)
	parser.add_argument('--q_lr', help='learning rate of value network', type=np.float32, default=1e-3)
	parser.add_argument('--act_l2', help='quadratic penalty on actions', type=np.float32, default=1.0)
	parser.add_argument('--polyak', help='interpolation factor in polyak averaging for DDPG', type=np.float32, default=0.95)

	parser.add_argument('--epochs', help='number of epochs', type=np.int32, default=20)
	parser.add_argument('--cycles', help='number of cycles per epoch', type=np.int32, default=20)
	parser.add_argument('--episodes', help='number of episodes per cycle', type=np.int32, default=50)
	parser.add_argument('--timesteps', help='number of timesteps per episode', type=np.int32, default=(50 if args.env[:5]=='Fetch' else 100))
	parser.add_argument('--train_batches', help='number of batches to train per episode', type=np.int32, default=20)

	parser.add_argument('--buffer_size', help='number of episodes in replay buffer', type=np.int32, default=10000)
	parser.add_argument('--buffer_type', help='type of replay buffer / whether to use Energy-Based Prioritization', type=str, default='energy', choices=['normal','energy'])
	parser.add_argument('--batch_size', help='size of sample batch', type=np.int32, default=256)
	parser.add_argument('--warmup', help='number of timesteps for buffer warmup', type=np.int32, default=10000)
	parser.add_argument('--her', help='type of hindsight experience replay', type=str, default='future', choices=['none', 'final', 'future'])
	parser.add_argument('--her_ratio', help='ratio of hindsight experience replay', type=np.float32, default=0.8)
	parser.add_argument('--pool_rule', help='rule of collecting achieved states', type=str, default='full', choices=['full', 'final'])

	parser.add_argument('--hgg_c', help='weight of initial distribution in flow learner', type=np.float32, default=3.0)
	parser.add_argument('--hgg_L', help='Lipschitz constant', type=np.float32, default=5.0)
	parser.add_argument('--hgg_pool_size', help='size of achieved trajectories pool', type=np.int32, default=1000)

	parser.add_argument('--save_acc', help='save successful rate', type=str2bool, default=True)

	parser.add_argument('--play_policy', help='Policy to choose an action', type=str, default='MPCRLPolicy',
						choices=['MPCRLPolicy', 'MPCPolicy', 'RLPolicy'])
	parser.add_argument('--mpc_gen', help='Generate MPC', type=str2bool, default=False)
	parser.add_argument('--mpc_path', help='MPC solver path', type=str, default='mpc/')
	parser.add_argument('--env_n_substeps', help='Steps to simulate', type=np.int32, default=20)
	parser.add_argument('--test_col_tolerance', help='Collision tolerance for test', type=np.int32, default=0)
	parser.add_argument('--test_run_id', help='Run id for test', type=np.int32, default=0)

	# GHGG
	parser.add_argument('--stop_hgg_threshold',
						help='threshold of goals inside goalspace, between 0 and 1, deactivated by default value 2!',
						type=np.float32, default=2)
	parser.add_argument('--n_x', help='number of vertices in x-direction for g-hgg', type=int, default=31)
	parser.add_argument('--n_y', help='number of vertices in y-direction for g-hgg', type=int, default=31)
	parser.add_argument('--n_z', help='number of vertices in z-direction for g-hgg', type=int, default=11)
	parser.add_argument('--show_goals', help='number of goals to show', type=np.int32, default=0)

	args = parser.parse_args()
	args.num_vertices = [args.n_x, args.n_y, args.n_z]
	args.goal_based = (args.env in Robotics_envs_id)
	args.clip_return_l, args.clip_return_r = clip_return_range(args)

	logger_name = args.alg+'-'+args.env+'-'+args.learn
	if args.tag!='': logger_name = args.tag+'-'+logger_name
	args.logger = get_logger(logger_name)

	for key, value in args.__dict__.items():
		if key!='logger':
			args.logger.info('{}: {}'.format(key,value))

	args.reward_dims = 1

	return args

def experiment_setup(args):
	env = make_env(args)
	env_test = make_env(args)
	if args.goal_based:
		args.obs_dims = list(goal_based_process(env.reset()).shape)
		args.acts_dims = [env.action_space.shape[0]]
		args.compute_reward = env.compute_reward
		args.compute_distance = env.compute_distance

	args.buffer = buffer = ReplayBuffer_Episodic(args)
	args.learner = learner = create_learner(args)
	args.agent = agent = create_agent(args)
	args.logger.info('*** network initialization complete ***')
	args.tester = tester = Tester(args)
	args.logger.info('*** tester initialization complete ***')

	return env, env_test, agent, buffer, learner, tester

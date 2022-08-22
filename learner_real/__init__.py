from .normal import NormalLearner
from .hgg import HGGLearner
from .ghgg import GHGGLearner

learner_collection = {
	'normal': NormalLearner,
	'hgg': HGGLearner,
	'ghgg': GHGGLearner,
}

def create_learner(args):
	return learner_collection[args.learn](args)
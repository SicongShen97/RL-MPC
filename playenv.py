import envs
from gym.wrappers.monitoring.video_recorder import VideoRecorder
import gym
from env_ext import make_env
from common import get_args

env = gym.make("MountainCar-v0")
recorder = VideoRecorder(env)

recorder.close()

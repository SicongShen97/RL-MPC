from typing import List

import numpy as np
import tensorflow as tf
tf.compat.v1.disable_eager_execution()
import os

from algorithm.replay_buffer import goal_based_process
from env_ext.fetch import MPCControlGoalEnv
from policies.policy import Policy


class RLPolicy(Policy):
    Vector = List[np.ndarray]
    InfoVector = List[dict]

    def __init__(self, args):
        # get current policy from path (restore tf session + graph)
        self.play_dir = args.play_path
        self.play_epoch = args.play_epoch
        self.meta_path = os.path.join(self.play_dir, "saved_policy-{}.meta".format(self.play_epoch))
        self.checkpoint_path = os.path.join(self.play_dir, "saved_policy-{}".format(self.play_epoch))
        self.sess = tf.compat.v1.Session()
        self.saver = tf.compat.v1.train.import_meta_graph(self.meta_path)
        self.saver.restore(self.sess, self.checkpoint_path)
        graph = tf.compat.v1.get_default_graph()
        self.raw_obs_ph = graph.get_tensor_by_name("raw_obs_ph:0")
        self.pi = graph.get_tensor_by_name("main/policy/net/pi/Tanh:0")
        print('Meta path: ', self.meta_path, self.checkpoint_path)

    def reset(self):
        return

    # predicts next actions for given states (observations)
    def predict(self, obs: Vector) -> (Vector, InfoVector):
        actions = self._my_step_batch(obs)
        return actions, []

    def _my_step_batch(self, obs):
        # compute actions from obs based on current policy by running tf session initialized before
        obs = [goal_based_process(ob) for ob in obs]
        actions = self.sess.run(self.pi, {self.raw_obs_ph: obs})
        return actions

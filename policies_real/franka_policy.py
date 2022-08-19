from abc import abstractmethod
from typing import List

import numpy as np

from env_ext.fetch import MPCControlGoalEnv


class Policy:
    Vector = List[np.ndarray]
    InfoVector = List[dict]

    envs = []

    def set_envs(self, envs: List[MPCControlGoalEnv]):
        # environments references
        self.envs = envs

    @abstractmethod
    def predict(self, obs: Vector) -> (Vector, InfoVector):
        """Predict the next step"""
        raise NotImplementedError

    @abstractmethod
    def initial_info(self, obs: Vector) -> InfoVector:
        """Get the initial information for debugging"""
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Predict the next step"""
        raise NotImplementedError

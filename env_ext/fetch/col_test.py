import gym
import numpy as np
from .vanilla import VanillaGoalEnv


class ColTestGoalEnv(VanillaGoalEnv):

    collisions = 0

    def __init__(self, args):
        VanillaGoalEnv.__init__(self, args)

    def step(self, action):
        # Extend the observation with a collision information
        obs, reward, done, info = super().step(action)
        obs = self.extend_obs(obs)
        #reward = self.extend_reward(reward, obs)
        return obs, reward, done, info

    # def extend_reward(self, reward, obs):
    #     new_reward = reward
    #     if obs['collision_check']:
    #         new_reward = -5.0
    #     return new_reward

    def extend_obs(self, obs):
        sim = self.sim
        exists_collision = False
        for i in range(sim.data.ncon):
            contact = sim.data.contact[i]

            for obstacle_id in self.sim_env.geom_ids_obstacles:
                if (contact.geom1 == obstacle_id) or (contact.geom2 == obstacle_id):
                    exists_collision = True
                    break
        obs['collision_check'] = exists_collision
        if exists_collision:
            self.collisions += 1
        return obs

    def reset(self):
        self.collisions = 0
        return super().reset()


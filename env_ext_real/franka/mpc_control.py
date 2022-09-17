import gym
import numpy as np
from .vanilla import VanillaGoalEnv


class MPCControlGoalEnv(VanillaGoalEnv):
    total_reward = 0
    collisions = 0
    obj_distance_threshold = 0.03

    def __init__(self, args):
        VanillaGoalEnv.__init__(self, args)

    def get_obs(self):
        obs = super().get_obs()
        obs = self.extend_obs(obs, self.prev_obs)
        return obs

    def disable_action_limit(self):
        self.env.env.env.limit_action = 1.0

    @property
    def time(self):
        return self.sim.get_state().time

    def step(self, action):
        # Extend the observation with a collision information
        prev_obs = self.prev_obs
        obs, reward, done, info = super().step(action)
        obs = self.extend_obs(obs, prev_obs)
        reward = self._extend_reward(reward, obs)
        info = self._extend_info(info)
        return obs, reward, done, info

    def _extend_info(self, info):
        new_info = info.copy()
        new_info['Collisions'] = self.collisions
        new_info['ExReward'] = self.total_reward
        return new_info


    def _extend_reward(self, reward, obs):
        new_reward = reward
        if obs['collision_check']:
            new_reward = -4.0
        if obs['object_dis'] > self.obj_distance_threshold:
            new_reward = -10.0  # object is no more in the grip

        self.total_reward += new_reward

        return new_reward

    def extend_obs(self, obs, prev_obs):
        sim = self.sim
        exists_collision = False
        object_id = self.sim_env.geom_id_object
        for i in range(sim.data.ncon):
            contact = sim.data.contact[i]

            for obstacle_id in self.sim_env.geom_ids_obstacles:
                if (contact.geom1 == obstacle_id) or (contact.geom2 == obstacle_id):
                    exists_collision = True
                    break
        obs['collision_check'] = exists_collision
        if exists_collision:
            self.collisions += 1

        # calculate the velocities of the objects
        real_obstacle_info = obs['real_obstacle_info']
        real_obstacle_info_pos = real_obstacle_info[:,0:3]
        prev_real_obstacle_info_pos = prev_obs['real_obstacle_info'][:,0:3]
        dt = self.dt
        obj_vels = (real_obstacle_info_pos - prev_real_obstacle_info_pos) / dt
        obs['obj_vels'] = obj_vels
        ob = obs['observation'].copy()

        # extend the observation with obstacle positions and velocities
        obs['observation'] = np.concatenate([ob, real_obstacle_info.ravel(), obj_vels.ravel()])

        return obs

    def reset(self):
        self.collisions = 0
        self.total_reward = 0
        obs = super().reset()
        obs = self.extend_obs(obs, self.prev_obs)
        return obs

    @property
    def dt(self):
        return self.env.env.dt

    # MPC methods
    # ----------------------------

    def subgoal(self, rl_action: np.ndarray, obs) -> np.ndarray:
        action = np.clip(rl_action, self.action_space.low, self.action_space.high)
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        pos_ctrl *= 0.05  # limit maximum change in position
        # try:
        #     grip_pos = self.sim.data.get_site_xpos('robot0:grip')
        # except:
        #     grip_pos = self.sim.data.get_site_xpos('grip_site')
        grip_pos = obs['observation'][:3]

        sub_goal = grip_pos + pos_ctrl

        if self.sim_env.block_z:
            target_z = sub_goal[2]
            if target_z > self.sim_env.block_max_z:
                # robot can not move higher
                sub_goal[2] = max(0, self.sim_env.block_max_z)

        return sub_goal

    def extract_parameters_3d(self, horizon: int, ts: float, sub_goal: np.ndarray):
        goal = self.goal
        sub_goal = sub_goal
        t = self.time
        dt = ts if ts is not None else self.dt
        N = horizon
        n = self.env.n_obstacles
        positions = np.array([self._extract_obstacles(time=t + dt * i) for i in range(N)])
        positions = np.reshape(positions, (-1, n * 6))
        n_obst = positions.shape[0]

        # add goal to all timesteps
        sub_goal_reshaped = np.reshape(sub_goal, (1, 3))
        goal_reshaped = np.reshape(goal, (1, 3))
        goals = np.repeat(goal_reshaped, n_obst, axis=0)
        sub_goals = np.repeat(sub_goal_reshaped, n_obst, axis=0)
        parameters = np.hstack((sub_goals, goals, positions))

        return parameters

    def _extract_obstacles(self, time):
        # new interface
        obstacles_list = self.env.get_obstacles(time=time)
        return np.array(obstacles_list).flatten()

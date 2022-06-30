import os
import gym
import numpy as np
from gym_robotics.envs import rotations, robot_env, utils
import math
import copy

# Ensure we get the path separator correct on windows
MODEL_XML_PATH = os.path.join(os.getcwd(), 'panda_envs', 'assets', 'fetch', 'panda_pick_dyn_labyrinth.xml')

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


class PandaFetchPickDynLabyrinthEnv(robot_env.RobotEnv, gym.utils.EzPickle):
    def __init__(self, reward_type='sparse', n_substeps=20):

        """Initializes a new Fetch environment.

        Args:
            model_path (string): path to the environments XML file
            n_substeps (int): number of substeps the simulation runs on every call to step
            gripper_extra_height (float): additional height above the table when positioning the gripper
            block_gripper (boolean): whether or not the gripper is blocked (i.e. not movable) or not
            has_object (boolean): whether or not the environment has an object
            target_in_the_air (boolean): whether or not the target should be in the air above the table or on the table surface
            target_offset (float or array with 3 elements): offset of the target
            obj_range (float): range of a uniform distribution for sampling initial object positions
            target_range (float): range of a uniform distribution for sampling a target
            distance_threshold (float): the threshold after which a goal is considered achieved
            initial_qpos (dict): a dictionary of joint names and values that define the initial configuration
            reward_type ('sparse' or 'dense'): the reward type, i.e. sparse or dense
        """
        initial_qpos = {
            # 'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
            'panda0_joint1': -2.24,
            'panda0_joint2': -0.038,
            'panda0_joint3': 2.55,
            'panda0_joint4': -2.68,
            'panda0_joint5': 0.0,
            'panda0_joint6': 0.984,
            'panda0_joint7': 0.0327,

        }
        model_path = MODEL_XML_PATH
        n_substeps = 20
        self.further= False
        self.gripper_extra_height = [0, 0, 0]
        self.block_gripper = True
        self.has_object = True
        self.block_object_in_gripper = True
        self.block_z = True
        self.target_in_the_air = False
        self.target_offset = 0.0
        self.obj_range = 0.06 # originally 0.15
        self.target_range = 0.06
        self.target_range_x = 0.06 # entire table: 0.125
        self.target_range_y = 0.06 # entire table: 0.175
        self.distance_threshold = 0.05
        self.reward_type = reward_type
        self.limit_action = 0.05  # limit maximum change in position

        self.adapt_dict=dict()
        self.adapt_dict["field"] = [1.35, 0.75, 0.43, 0.3, 0.35, 0.03]
        self.adapt_dict["obstacles"] = [[1.3 - 0.1, 0.75, 0.43, 0.11, 0.02, 0.03],
                                    [1.3 - 0.23, 0.75, 0.43, 0.02, 0.35, 0.03],
                                    [1.3 + 0.03, 0.75, 0.43, 0.02, 0.2, 0.03]]
        self.dyn_obstacles = [[1.5, 0.60, 0.435, 0.03, 0.03, 0.03], [1.5, 0.80, 0.435, 0.03, 0.03, 0.03]]

        super(PandaFetchPickDynLabyrinthEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=4,
            initial_qpos=initial_qpos)

        gym.utils.EzPickle.__init__(self)
        self._setup_obstacles()

    def _setup_obstacles(self):
        # setup velocity limits
        self.vel_lims = np.array([0.2, 0.4])
        self.n_moving_obstacles = len(self.dyn_obstacles)
        self.current_obstacle_vels = []

        self._setup_dyn_limits()

        self.obstacle_slider_idxs = []
        self.obstacle_slider_idxs.append(self.sim.model.joint_names.index('dyn_obstacle:joint'))
        self.obstacle_slider_idxs.append(self.sim.model.joint_names.index('dyn_obstacle2:joint'))

        self.geom_id_object = self.sim.model.geom_name2id('object0')

        self.geom_ids_obstacles = []
        for name in ['obstacle_1', 'obstacle_2', 'obstacle_3', 'dyn_obstacle:geom', 'dyn_obstacle2:geom']:
            self.geom_ids_obstacles.append(self.sim.model.geom_name2id(name))

    def _setup_dyn_limits(self):
        self.pos_difs = []

        # assume all obstacles are moving vertically
        for _ in self.dyn_obstacles:
            self.pos_difs.append(0.15 - 0.03)

    def set_obstacle_slide_pos(self, positions):
        qpos = self.sim.data.qpos.flat[:]
        for i in range(self.n_moving_obstacles):
            # move obstacles
            pos = positions[i]
            qpos[self.obstacle_slider_idxs[i]] = pos
        to_mod = copy.deepcopy(self.sim.get_state())
        to_mod = to_mod._replace(qpos=qpos)
        self.sim.set_state(to_mod)
        self.sim.forward()

    def set_obstacle_slide_vel(self, velocities):
        qvel = self.sim.data.qvel.flat[:]
        for i, vel in enumerate(velocities):
            qvel[self.obstacle_slider_idxs[i]] = vel
        to_mod = copy.deepcopy(self.sim.get_state())
        to_mod = to_mod._replace(qvel=qvel)
        self.sim.set_state(to_mod)
        self.sim.forward()

    def compute_obstacle_rel_x_positions(self, time) -> np.ndarray:
        n = self.n_moving_obstacles
        new_positions = np.zeros(n)
        t = time

        for i in range(self.n_moving_obstacles):
            max_q = self.pos_difs[i]
            s_q = max_q * 2
            v = self.current_obstacle_vels[i]
            a = max_q  # amplitude
            p = s_q / v  # period
            s = self.current_obstacle_shifts[i] * 2 * math.pi  # time shift
            new_pos_x = 2 * a / math.pi * math.asin(math.sin(s + 2 * math.pi / p * t))  # triangle wave
            new_positions[i] = new_pos_x

        return new_positions

    def compute_obstacle_positions(self, time) -> np.ndarray:
        t = time
        n = self.n_moving_obstacles
        new_positions_x = self.compute_obstacle_rel_x_positions(time=t)
        new_positions = np.zeros(n * 2)
        obst = self.dyn_obstacles

        for i in range(n):
            new_positions[2 * i] = obst[i][0] + new_positions_x[i]
            new_positions[2 * i + 1] = obst[i][1]

        return new_positions


    def move_obstacles(self):
        t = self.sim.get_state().time + self.dt
        new_positions_x = self.compute_obstacle_rel_x_positions(time=t)
        self.set_obstacle_slide_pos(new_positions_x)

    # GoalEnv methods
    # ----------------------------

    def compute_reward(self, achieved_goal, goal, info): # leave unchanged
        # Compute distance between goal and the achieved goal.
        d = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d
    """    
    def is_inside_goal_space(self, goal):
        assert goal.shape == (3,)
        if (self.target_center[0] - self.target_range_x <= goal[0] <= self.target_center[0] + self.target_range_x) and 
            (self.target_center[1] - self.target_range_y <= goal[1] <= self.target_center[1] + self.target_range_y) and
            (self.target_center[2] )
    """
    # RobotEnv methods
    # ----------------------------

    def step(self, action):
        self.move_obstacles()
        return super(PandaFetchPickDynLabyrinthEnv, self).step(action)

    def _step_callback(self):
        # initially close gripper
        # if self.block_gripper:
        #     self.sim.data.set_joint_qpos('robot0:l_gripper_finger_joint', 0.)
        #     self.sim.data.set_joint_qpos('robot0:r_gripper_finger_joint', 0.)
        #     self.sim.forward()

        if self.block_object_in_gripper and self.block_gripper:
            self.sim.data.set_joint_qpos('finger_joint1', 0.019)
            self.sim.data.set_joint_qpos('finger_joint2', 0.019)
            self.sim.forward()

    def _set_action(self, action):
        assert action.shape == (4,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        pos_ctrl *= self.limit_action  # limit maximum change in position
        rot_ctrl = [0, 1., 0., 0.]  # fixed rotation of the end effector, expressed as a quaternion
        gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        assert gripper_ctrl.shape == (2,)
        if self.block_gripper:
            gripper_ctrl = np.zeros_like(gripper_ctrl)
        if self.block_z:
            pos_ctrl[2] = 0.
        action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        utils.ctrl_set_action(self.sim, action)
        utils.mocap_set_action(self.sim, action)
        self.sim.data.mocap_pos[0][-1] = 0.4 #make the gripper

    def _get_obs(self):
        # positions
        grip_pos = self.sim.data.get_site_xpos('grip_site')
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        grip_velp = self.sim.data.get_site_xvelp('grip_site') * dt
        robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
        if self.has_object:
            object_pos = self.sim.data.get_site_xpos('object0')
            # rotations
            object_rot = rotations.mat2euler(self.sim.data.get_site_xmat('object0'))
            # velocities
            object_velp = self.sim.data.get_site_xvelp('object0') * dt
            object_velr = self.sim.data.get_site_xvelr('object0') * dt
            # gripper state
            object_rel_pos = object_pos - grip_pos
            object_velp -= grip_velp
        else:
            object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
        gripper_state = robot_qpos[-2:]
        gripper_vel = robot_qvel[-2:] * dt  # change to a scalar if the gripper is made symmetric

        if not self.has_object:
            achieved_goal = grip_pos.copy()
        else:
            achieved_goal = np.squeeze(object_pos.copy())

        body_id = self.sim.model.body_name2id('dyn_obstacle')
        pos1 = np.array(self.sim.data.body_xpos[body_id].copy())
        body_id2 = self.sim.model.body_name2id('dyn_obstacle2')
        pos2 = np.array(self.sim.data.body_xpos[body_id2].copy())
        dims = self.dyn_obstacles[0][3:6]
        ob1 = np.concatenate((pos1, dims.copy()))
        ob2 = np.concatenate((pos2, dims.copy()))

        obs = np.concatenate([
            grip_pos, object_pos.ravel(), object_rel_pos.ravel(), gripper_state, object_rot.ravel(),
            object_velp.ravel(), object_velr.ravel(), grip_velp, gripper_vel, ob1, ob2
        ])
        obj_dist = np.linalg.norm(object_rel_pos.ravel()[:2]) # only xy

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
            'real_obstacle_info': np.array([ob1, ob2]),
            'object_dis': obj_dist,
            #'object_pos': object_pos,
            #'grip_pos': grip_pos
        }

    def _viewer_setup(self):
        body_id = self.sim.model.body_name2id('right_gripper')
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 130.
        self.viewer.cam.elevation = -24.
        self.viewer._run_speed = 0.3

    def _render_callback(self):
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = self.sim.model.site_name2id('target0')
        self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
        self.sim.forward()

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)

        # Randomize start position of object.
        if self.has_object and not self.block_object_in_gripper:
            object_xpos = self.initial_gripper_xpos[:2]
            object_xpos = self.initial_gripper_xpos[:2] + self.np_random.uniform(-self.obj_range, self.obj_range,
                                                                                 size=2)
            object_qpos = self.sim.data.get_joint_qpos('object0:joint')
            assert object_qpos.shape == (7,)
            object_qpos[:2] = object_xpos
            self.sim.data.set_joint_qpos('object0:joint', object_qpos)

        if self.block_object_in_gripper:
            object_xpos = self.initial_gripper_xpos[:2]
            object_qpos = self.sim.data.get_joint_qpos('object0:joint')
            assert object_qpos.shape == (7,)
            object_qpos[:2] = object_xpos
            self.sim.data.set_joint_qpos('object0:joint', object_qpos)
            # open the gripper
            if self.block_gripper:
                self.sim.data.set_joint_qpos('finger_joint1', 0.019)
                self.sim.data.set_joint_qpos('finger_joint2', 0.019)
            else:
                self.sim.data.set_joint_qpos('finger_joint1', 0.05)
                self.sim.data.set_joint_qpos('finger_joint2', 0.05)

        # randomize obstacles
        directions = self.np_random.choice([-1, 1], size=2)
        self.current_obstacle_shifts = self.np_random.uniform(0, 1.0, size=2)
        self.current_obstacle_vels = directions * self.np_random.uniform(self.vel_lims[0], self.vel_lims[1], size=2)
        self.move_obstacles()  # move obstacles to the initial positions

        self.sim.forward()
        return True

    def _sample_goal(self):
        goal = self.target_center.copy()

        goal[1] += self.np_random.uniform(-self.target_range_y, self.target_range_y)
        goal[0] += self.np_random.uniform(-self.target_range_x, self.target_range_x)

        return goal.copy()

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        utils.reset_mocap_welds(self.sim)

        # initial markers (index 3 is arbitrary!)
        self.target_center = self.sim.data.get_site_xpos('target_center')
        self.init_center = self.sim.data.get_site_xpos('init_center')
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()[4]

        # Move end effector into position.
        gripper_target = self.init_center + self.gripper_extra_height #+ self.sim.data.get_site_xpos('robot0:grip')
        gripper_rotation = np.array([0, 1., 0., 0.])
        self.sim.data.set_mocap_pos('panda0:mocap', gripper_target)
        self.sim.data.set_mocap_quat('panda0:mocap', gripper_rotation)
        for _ in range(10):
            self.sim.step()
        # Extract information for sampling goals.
        self.initial_gripper_xpos = self.sim.data.get_site_xpos('grip_site').copy()
        object_xpos = self.initial_gripper_xpos
        object_xpos[2] = 0.2  # table height

        if self.block_object_in_gripper:
            # place object in the gripper
            object_xpos2 = self.initial_gripper_xpos[:2]
            object_qpos2 = self.sim.data.get_joint_qpos('object0:joint')
            object_qpos2[:2] = object_xpos2
            object_qpos2[2] += 0.02
            self.sim.data.set_joint_qpos('object0:joint', object_qpos2)

        site_id = self.sim.model.site_name2id('init_1')
        self.sim.model.site_pos[site_id] = object_xpos + [self.obj_range, self.obj_range, 0.0] - sites_offset
        site_id = self.sim.model.site_name2id('init_2')
        self.sim.model.site_pos[site_id] = object_xpos + [self.obj_range, -self.obj_range, 0.0] - sites_offset
        site_id = self.sim.model.site_name2id('init_3')
        self.sim.model.site_pos[site_id] = object_xpos + [-self.obj_range, self.obj_range, 0.0] - sites_offset
        site_id = self.sim.model.site_name2id('init_4')
        self.sim.model.site_pos[site_id] = object_xpos + [-self.obj_range, -self.obj_range, 0.0] - sites_offset

        site_id = self.sim.model.site_name2id('mark1')
        self.sim.model.site_pos[site_id] = self.target_center + [self.target_range_x, self.target_range_y, 0.0] - sites_offset
        site_id = self.sim.model.site_name2id('mark2')
        self.sim.model.site_pos[site_id] = self.target_center + [-self.target_range_x, self.target_range_y, 0.0] - sites_offset
        site_id = self.sim.model.site_name2id('mark3')
        self.sim.model.site_pos[site_id] = self.target_center + [self.target_range_x, -self.target_range_y, 0.0] - sites_offset
        site_id = self.sim.model.site_name2id('mark4')
        self.sim.model.site_pos[site_id] = self.target_center + [-self.target_range_x, -self.target_range_y, 0.0] - sites_offset

        self.sim.step()
        if self.has_object:
            self.height_offset = self.sim.data.get_site_xpos('object0')[2]

    def render(self, mode='human', width=1080, height=1080):
        return super(PandaFetchPickDynLabyrinthEnv, self).render(mode, width, height)



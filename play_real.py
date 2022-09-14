import numpy as np
import os
from env_ext_real import make_env
from algorithm.replay_buffer import goal_based_process
from mpc_real import MPCDebugPlot
from policies_real import MPCPolicy, RLPolicy, Policy, make_policy
from common_real import get_args
from gym.wrappers.monitoring.video_recorder import VideoRecorder
from envs_real import register_custom_envs
from camera_librealsense import Camera
from franka_robot import FrankaRobot
import sys
import time


class Player:
    policy: Policy = None
    debug_plot: MPCDebugPlot = None
    # debug_plot: MPCDebugPlotSmall = None
    mpc_policy = False

    obst_sizes = {"FrankaPickDynSqrObstacles-v1": np.array([[0.015, 0.017], [0.015, 0.017]]),
                  "FrankaPickDynObstacles-v1": np.array([[0.045, 0.017], [0.015, 0.017]])}
    obst_vels = {"FrankaPickDynSqrObstacles-v1": np.array([0.02, 0.03]),
                 "FrankaPickDynObstacles-v1": np.array([0.0, 0.03])}

    def __init__(self, args):
        # mujoco env set
        self.env = make_env(args)
        # policy set
        self.policy = make_policy(args)
        self.policy.set_env(env=self.env)
        # robot set
        self.robot = FrankaRobot("192.168.5.12")
        self.gripper = self.robot.gripper
        self.block_z = False if args.env == 'FrankaPickDynLiftedObstacles-v1' else True
        # real env set
        self.offset = np.array([0.8, 0.75])  # robot base relative to the origin in simulator
        self.goal = np.array([0.5, -0.3]) + self.offset
        self.subgoal = self.goal + self.offset
        self.init = np.array([0.5, 0.3])
        self.dt = 0.5  # time interval in real env
        self.length = 80  # number of steps to take
        self.obst_size = self.obst_sizes[args.env]  # (x/2, y/2)
        self.vels = self.obst_vels[args.env]  # velocity of obstacle
        self.obst_rel_robot = np.array([[0.5, 0.1], [0.5, -0.1]])  # middle pose relative to robot base
        self.pos_dif = 0.1
        self.center_x = 0.5 + self.offset[0]
        self.origin_offset = np.array([0.042, 0.041])
        self.pre_dists = np.array([None, None])
        self.signs = np.array([1, 1])
        # camera set
        self.camera = Camera()

    def initialize(self):
        # start camera
        self.camera.start()
        # move robot to initial pose
        self.robot.move([0, 0, 0.2])
        z = self.robot.current_pose()[2]
        pose = np.append(self.init, z)
        self.robot.move_to_init(pose)
        input("Enter to lower the gripper.")
        pose = np.append(self.init, 0.02)
        self.robot.move_to_init(pose)
        input("Enter to grasp the object.")
        self.robot.clamp()
        self.robot.move([0, 0, 0.15])
        input("Enter to start moving.")

    def get_obs_distance(self):
        frame = self.camera.get_frame()
        try:
            dists, _ = self.camera.get_distance(frame, add_to_frame=False)
        except:
            dists = self.origin_offset
        dists -= self.origin_offset  # relative to origin
        return dists

    def get_obs_pose(self, dists):
        # get obstacle pose
        dyn_obstacles = []
        for i in range(len(dists)):
            dyn_obstacle = np.append(self.obst_rel_robot[i]+self.offset, self.obst_size[i])
            dyn_obstacle[0] += dists[i] - self.pos_dif
            dyn_obstacles.append(dyn_obstacle)
        return np.array(dyn_obstacles)

    def finish(self):
        self.camera.stop()
        cur_pose = self.robot.current_pose()
        disp_z = cur_pose[2] - 0.02
        self.robot.move([0, 0, -disp_z])
        self.robot.release()

    def close_to_goal(self, xinit):
        # cur_pos = self.robot.current_pose()[:2]
        goal = self.goal
        if np.linalg.norm(xinit[:2] - goal[:2], 2) <= 0.05:
            return True
        return False

    def play(self):
        xinit = self.init + self.offset
        dists = self.get_obs_distance()
        ob = self.env.reset()
        print("original obs", ob)
        ob = self.set_obs(ob, xinit, dists)
        if args.play_policy in ['MPCPolicy']:
            self.policy.set_sub_goal(ob['desired_goal'])

        while not self.close_to_goal(xinit):
            # print("xinit: ", xinit)
            t1 = time.time()
            # print("observation", ob["observation"])
            real_actions = self.policy.predict([ob])
            real_action = real_actions[0]
            if args.play_policy in ['RLPolicy']:
                real_action *= 0.05
                # for xy motion
                real_action[2] = 0
            self.robot.move(real_action[:3])
            print("action:", real_action)

            # update the original pose
            cur_pose = self.robot.current_pose()
            x_init = cur_pose[0] + self.offset[0]
            y_init = cur_pose[1] + self.offset[1]
            xinit = np.array([x_init, y_init])
            dists = self.get_obs_distance()
            ob = self.set_obs(ob, xinit, dists)
            print("time:", time.time() - t1)
            if self.dt - (time.time() - t1) > 0:
                time.sleep(self.dt - (time.time() - t1))
        self.finish()

    def set_obs(self, obs, xinit, dists):
        obs["observation"][:2] = xinit[:2]
        obs["observation"][3:5] = xinit[:2]

        dyn_obstacles = self.get_obs_pose(dists)
        obs["observation"][9:11] = dyn_obstacles[0, :2]
        obs["observation"][15:17] = dyn_obstacles[1, :2]
        obs["real_obstacle_info"][0, :2] = dyn_obstacles[0, :2]
        obs["real_obstacle_info"][1, :2] = dyn_obstacles[1, :2]

        if self.pre_dists.any():
            signs = np.sign(dists - self.pre_dists)
            self.signs[0] = signs[0] if signs[0] != 0 else -self.signs[0]
            self.signs[1] = signs[1] if signs[1] != 0 else -self.signs[1]
        self.pre_dists = dists
        obs["observation"][21] = self.vels[0]*self.signs[0]
        obs["observation"][24] = self.vels[1]*self.signs[1]
        obs["obj_vels"][0, 0] = self.vels[0]*self.signs[0]
        obs["obj_vels"][1, 0] = self.vels[1]*self.signs[1]

        goal = self.goal
        obs["desired_goal"][:2] = goal[:2]

        obs["dt"] = self.dt
        obs["pos_dif"] = self.pos_dif
        obs["center_x"] = self.center_x

        return obs


if __name__ == "__main__":
    register_custom_envs()
    # Call play.py in order to see current policy progress
    args = get_args()
    player = Player(args)
    player.initialize()
    player.play()
    # player.record_video(raw_path="/home/ssc/bachelor-thesis/videos/rollouts_{}_{}".format(args.env, args.play_policy))

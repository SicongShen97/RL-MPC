from mpc_real import make_mpc
from typing import List, Union
from mpc_real.franka_mpc_common import extract_parameters
from camera_librealsense import Camera
from franka_robot import FrankaRobot
import time
import sys
import numpy as np
from policies_real import Policy


class MPCPolicy:
    # last_preds_x: List[Union[np.ndarray, None]] = []  # last predictions
    # last_preds_idx: List[Union[int, None]] = []  # last taken action
    # last_error: List[Union[bool]] = []  # last solve unfeasible

    obst_sizes = {"FrankaPickDynSqrObstacles-v1": np.array([[0.015, 0.017], [0.015, 0.017]])}
    obst_vels = {"FrankaPickDynSqrObstacles-v1": np.array([0.02, 0.03])}

    def __init__(self, args):
        model, solver, codeoptions = make_mpc(args)
        self.model = model
        self.solver = solver
        self.codeoptions = codeoptions
        # robot set
        self.robot = FrankaRobot("192.168.5.12")
        self.gripper = self.robot.gripper
        # real env set
        self.offset = np.array([0.8, 0.75])  # robot base relative to the origin in simulator
        self.goal = np.array([0.5, -0.3])
        self.subgoal = goal
        self.init = np.array([0.5, 0.3])
        self.dt = 0.5  # time interval in real env
        self.length = 80  # number of steps to take
        # obstacle set
        self.obst_size = self.obst_sizes[args.env] #  (x/2, y/2)
        self.vels = self.obst_vels[args.env]  # velocity of obstacle
        self.obst_rel_robot = np.array([[0.5, 0.1], [0.5, -0.1]])  # middle pose relative to robot base
        self.pos_dif = 0.1
        self.center_x = 0.5 + self.offset[0]
        self.origin_offset = np.array([0.042, 0.039])
        self.pre_dists = np.array([None, None])
        self.signs = np.array([1, 1])
        self.last_pred_u = None
        self.last_pred_idx = None
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
        self.robot.move([0, 0, 0.2])
        input("Enter to start moving.")
        # for mpc
        # init = self.init + self.offset
        # subgoal = self.subgoal + self.offset
        # goal = self.goal + self.offset
        # self.set_runtime_param(init, subgoal, goal)

    def get_problem(self, xinit, parameters):
        x0i = np.zeros(self.model.nvar)
        x0i[3:5] = xinit
        x0 = np.reshape(x0i, (self.model.nvar, 1))
        pred = np.repeat(x0, self.model.N, axis=1)  # first prediction corresponds to initial guess
        problem = {"x0": pred, "xinit": xinit,
                   "all_parameters": np.reshape(parameters, (self.model.npar * self.model.N, 1))}
        return problem

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

    def play(self):
        # pre_dists = np.array([None, None])
        # signs = np.array([1, 1])
        xinit = self.init + self.offset
        while not self.close_to_goal(xinit):
            # print("xinit: ", xinit)
            t1 = time.time()
            # predict
            real_action = self.predict(xinit)
            # predict
            self.robot.move([*real_action, 0])
            # print("action:", real_action)
            # update the original pose
            cur_pose = self.robot.current_pose()
            x_init = cur_pose[0] + self.offset[0]
            y_init = cur_pose[1] + self.offset[1]
            xinit = np.array([x_init, y_init])
            print("time:", time.time() - t1)
            if self.dt - (time.time() - t1) > 0:
                time.sleep(self.dt - (time.time() - t1))
        self.finish()

    def close_to_goal(self, xinit):
        # cur_pos = self.robot.current_pose()[:2]
        goal = self.goal + self.offset
        if np.linalg.norm(xinit - goal, 2) <= 0.01:
            return True
        return False

    def set_sub_goal(self, subgoal):
        # environments references for the MPC
        self.subgoal = subgoal

    def predict(self, xinit):
        last_pred_u = self.last_pred_u
        last_pred_idx = self.last_pred_idx

        # xinit = self.init + self.offset
        dists = self.get_obs_distance()
        if self.pre_dists.any():
            self.signs = np.sign(dists - self.pre_dists)
        self.pre_dists = dists

        dyn_obstacles = self.get_obs_pose(dists)
        goal = self.goal + self.offset
        subgoal = self.subgoal + self.offset

        if np.linalg.norm(xinit - goal) < self.model.N * self.dt * 1.0:
            # solve the MPC problem using the global goal as sub goal
            subgoal = goal

        parameters = extract_parameters(subgoal, goal, self.dt, self.model.N, dyn_obstacles, self.signs * self.vels,
                                        self.pos_dif,
                                        self.center_x)

        problem = self.get_problem(xinit, parameters)

        # call the solver
        # output, exitflag, info = self.solver.solve(problem)
        pred_u, pred_x, exitflag, info = self._solve_mpc(problem)
        if exitflag != 1:
            # take previous actions
            if last_pred_u is not None and last_pred_idx is not None and last_pred_idx < model.N - 1:
                action = last_pred_u[:, last_pred_idx + 1]
                self.last_pred_idx += 1  # remember action taken
            else:
                # stay still
                action = np.array([0, 0])
        else:
            action = pred_u[:, 0]
            # save last prediction in case of the error in the next step
            self.last_pred_u = pred_u
            self.last_pred_idx = 0

        # assert exitflag >= 0, "Problem happended in solver."
        # sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
        #                  .format(info.it, info.solvetime))

        # extract output of solver
        # temp = np.zeros((self.model.nvar, self.model.N))
        # for i in range(0, self.model.N):
        #     temp[:, i] = output['x{0:02d}'.format(i + 1)]
        # pred_u = temp[0:2, :]
        # pred_x = temp[3:5, :]
        #
        # real_action = pred_u[:, 0]

        return action

    def _solve_mpc(self, problem):
        model = self.model
        solver = self.solver
        output, exitflag, info = solver.solve(problem)

        # extract output of solver
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i + 1)]
        pred_u = temp[0:2, :]
        pred_x = temp[3:5, :]

        return pred_u, pred_x, exitflag, info




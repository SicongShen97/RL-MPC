from mpc_real import make_mpc
from typing import List, Union
from mpc_real.franka_mpc_common import extract_parameters
from camera_librealsense import Camera
from franka_robot import FrankaRobot
from policies_real.franka_policy import Policy
import time
import sys
import numpy as np


class MPCPolicy(Policy):

    def __init__(self, args):
        model, solver, codeoptions = make_mpc(args)
        self.model = model
        self.solver = solver
        self.codeoptions = codeoptions
        self.last_pred_u = None
        self.last_pred_idx = None
        self.subgoal = None
        self.pre_dists = np.array([None, None])
        # self.signs = np.array([1, 1])

    def reset(self):
        return

    def set_env(self, env):
        super().set_env(env)
        env.disable_action_limit()

    def get_problem(self, xinit, parameters):
        x0i = np.zeros(self.model.nvar)
        x0i[4:7] = xinit
        x0 = np.reshape(x0i, (self.model.nvar, 1))
        pred = np.repeat(x0, self.model.N, axis=1)  # first prediction corresponds to initial guess
        problem = {"x0": pred, "xinit": xinit,
                   "all_parameters": np.reshape(parameters, (self.model.npar * self.model.N, 1))}
        return problem

    def set_sub_goal(self, subgoal):
        # environments references for the MPC
        self.subgoal = subgoal

    def predict(self, obs):
        ob = obs[0]
        last_pred_u = self.last_pred_u
        last_pred_idx = self.last_pred_idx

        observation = ob['observation']
        obstacles = ob['real_obstacle_info']
        dt = ob['dt']
        vels = ob['obj_vels'][:, 0]
        pos_dif = ob['pos_dif']
        center_x = ob['center_x']

        grip_pos = observation[0:3]
        xinit = np.transpose(np.array([grip_pos[0], grip_pos[1], grip_pos[2]]))

        # dists = np.array([obstacles[0][0], obstacles[1][0]])
        # if self.pre_dists.any():
        #     self.signs = np.sign(dists - self.pre_dists)
        # self.pre_dists = dists

        # dyn_obstacles = self.get_obs_pose(dists)
        goal = ob["desired_goal"]
        subgoal = self.subgoal

        if np.linalg.norm(xinit[:2] - goal[:2]) < 0.3:
            # solve the MPC problem using the global goal as sub goal
            subgoal = goal

        parameters = extract_parameters(subgoal, goal, dt, self.model.N, obstacles, vels,
                                        pos_dif, center_x)

        problem = self.get_problem(xinit, parameters)

        # call the solver
        # output, exitflag, info = self.solver.solve(problem)
        pred_u, pred_x, exitflag, info = self._solve_mpc(problem)
        if exitflag < 0:
            # take previous actions
            if last_pred_u is not None and last_pred_idx is not None and last_pred_idx < self.model.N - 1:
                action = last_pred_u[:, last_pred_idx + 1]
                self.last_pred_idx += 1  # remember action taken
            else:
                # stay still
                action = np.array([0, 0, 0])
        else:
            action = pred_u[:, 0]
            # save last prediction in case of the error in the next step
            self.last_pred_u = pred_u
            self.last_pred_idx = 0

        # assert exitflag >= 0, "Problem happended in solver."
        # sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
        #                  .format(info.it, info.solvetime))

        return [action]

    def _solve_mpc(self, problem):
        model = self.model
        solver = self.solver
        output, exitflag, info = solver.solve(problem)

        # extract output of solver
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i + 1)]
        pred_u = temp[0:3, :]
        pred_x = temp[4:7, :]

        return pred_u, pred_x, exitflag, info




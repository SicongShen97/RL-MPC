from typing import List, Union
from env_ext.fetch import MPCControlGoalEnv
from mpc import make_mpc
import numpy as np

from policies.policy import Policy


class MPCPolicy(Policy):
    Vector = List[np.ndarray]
    InfoVector = List[dict]

    # MPC policy requires environment and sub goals
    sub_goals = []

    last_preds_x: List[Union[np.ndarray, None]] = []  # last predictions
    last_preds_idx: List[Union[int, None]] = []  # last taken action
    last_error: List[Union[bool]] = [] # last solve unfeasible

    grip_action = -0.8

    def __init__(self, args):
        model, solver, codeoptions = make_mpc(args)

        self.model = model
        self.solver = solver
        self.codeoptions = codeoptions
        self.integrator_ts = codeoptions.nlp.integrator.Ts
        self.agent_vel_x = 0
        self.agent_vel_y = 0
        self.agent_vel_z = 0
        self.grip_old_pos = None

    def reset(self):
        self.agent_vel_x = 0
        self.agent_vel_y = 0
        self.agent_vel_z = 0
        self.grip_old_pos = None
        self.last_preds_x = []
        self.last_preds_idx = []
        self.last_error = []


    def set_envs(self, envs: List[MPCControlGoalEnv]):
        super().set_envs(envs)
        for env in envs:
            env.disable_action_limit()

    def set_sub_goals(self, sub_goals: Vector):
        # environments references for the MPC
        self.sub_goals = sub_goals

        if len(self.last_preds_x) == 0:
            n_nodes = len(sub_goals)
            self.last_preds_x = [None for _ in range(n_nodes)]
            self.last_preds_idx = [None for _ in range(n_nodes)]
            self.last_error = [False for _ in range(n_nodes)]

    # predicts next actions for given states (observations)
    # requires envs and sub goals
    # envs and obs should be consistent
    def predict(self, obs: Vector) -> (Vector, InfoVector):
        actions, infos = self._my_step_batch(obs)
        return actions, infos

    def _my_step_batch(self, obs: Vector) -> (Vector, InfoVector):
        actions = []
        infos = []
        for i in range(len(obs)):
            ob = obs[i]
            a, info = self._predict(ob, i)
            actions.append(a)
            infos.append(info)

        return actions, infos

    # predict next action to the goal
    def _predict(self, ob: np.ndarray, i: int) -> (np.ndarray, dict):
        model = self.model
        solver = self.solver

        env = self.envs[i]
        sub_goal = self.sub_goals[i]

        last_pred_x = self.last_preds_x[i] if len(self.last_preds_x) > 0 else None
        last_pred_idx = self.last_preds_idx[i] if len(self.last_preds_idx) > 0 else None

        observation = ob['observation']
        obstacles = ob['real_obstacle_info']

        grip_pos = observation[0:3]

        # Set initial guess to start solver from
        xinit = np.transpose(np.array([grip_pos[0], grip_pos[1], grip_pos[2], self.agent_vel_x, self.agent_vel_y, self.agent_vel_z]))

        x0i = np.append(np.array([0., 0., 0., 0.]), xinit)
        x0 = np.transpose(np.tile(x0i, (1, model.N)))
        # Set initial condition

        goal_pos = env.goal
        if np.linalg.norm(grip_pos - goal_pos) < self.model.N * self.integrator_ts * 1.0:
            # solve the MPC problem using the global goal as sub goal
            mpc_goal = goal_pos
        else:
            mpc_goal = sub_goal

        parameters = env.extract_parameters_3d(horizon=model.N, ts=self.integrator_ts, sub_goal=mpc_goal)

        #print("init: ", xinit)
        #print("target: ", mpc_goal)

        problem = {'x0': x0,
                   'xinit': xinit,
                   'all_parameters': np.reshape(parameters, (model.npar * model.N, 1))}

        pred_u, pred_x, exitflag, info = self._solve_mpc(problem)

        # Apply optimized input u of first stage to system and save simulation data
        if exitflag != 1:
            # take previous prediction
            if last_pred_x is not None and last_pred_idx is not None and last_pred_idx < model.N - 1:
                #print('use last prediction: ', last_pred_idx + 1)
                next_x = last_pred_x[:, last_pred_idx + 1]
                self.last_preds_idx[i] += 1  # remember action taken
                self.last_error[i] = False
            else:
                # fallback to RL sub goal
                print('use RL next_x')
                #next_x = np.array([sub_goal[0], sub_goal[1], 0, 0])
                next_x = np.array([xinit[0], xinit[1], xinit[2], 0, 0, 0])
                self.last_error[i] = True
        else:
            next_x = pred_x[:, 1]
            # save last prediction in case of the error in the next step
            self.last_preds_x[i] = pred_x
            self.last_preds_idx[i] = 1
            self.last_error[i] = False

        xinit_pos = xinit[0:3]
        target_x = self._compute_target(next_x)

        self.agent_vel_x = next_x[3]
        self.agent_vel_y = next_x[4]
        self.agent_vel_z = next_x[5]
        self.old_ob = ob

        action_x = target_x - xinit_pos
        next_action = np.array([action_x[0], action_x[1], action_x[2], self.grip_action])

        info = {'exitflag': exitflag, 'horizon': model.N,
                'integrator_ts': self.integrator_ts,
                'info': info, 'pred_u': pred_u,
                'pred_x': pred_x, 'next_x': next_x, 'target_x': target_x,
                'parameters': parameters}

        return next_action, info

    def _compute_target(self, next_x) -> np.ndarray:
        # deceleration to zero velocity displacement
        next_x_vel = next_x[3:6]
        disp_t = 0.07
        next_x_pos = next_x[0:3]
        disp_x_pos = next_x_pos + next_x_vel / 2 * disp_t

        target_x = disp_x_pos

        return target_x

    def _solve_mpc(self, problem):
        model = self.model
        solver = self.solver
        output, exitflag, info = solver.solve(problem)

        # extract output of solver
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:d}'.format(i + 1)]
        pred_u = temp[0:4, :]
        pred_x = temp[4:10, :]

        return pred_u, pred_x, exitflag, info

    def initial_info(self, obs: Vector) -> InfoVector:
        infos = []
        for (ob, env) in zip(obs, self.envs):
            infos.append(self._initial_info(ob, env))
        return infos

    def _initial_info(self, ob: np.ndarray, env: MPCControlGoalEnv) -> dict:
        self.old_ob = ob
        infos = []
        xpos = ob['observation'][0:3]
        xinit = np.array([xpos[0], xpos[1], xpos[2], 0, 0, 0])
        goal = ob['desired_goal'][0:3]
        x0i = np.zeros(10)
        x0i[4:10] = xinit
        x0 = np.reshape(x0i, (10, 1))
        pred = np.repeat(x0, self.model.N, axis=1)  # first prediction corresponds to initial guess
        parameters = env.extract_parameters_3d(horizon=self.model.N,
                                            ts=self.integrator_ts,
                                            sub_goal=goal)
        return {'xinit': xinit, 'pred_x': pred[4:10], 'pred_u': pred[0:4], 'parameters': parameters}

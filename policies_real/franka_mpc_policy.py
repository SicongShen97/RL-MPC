from mpc_real.franka_pick_dyn_obstacle import generate_pathplanner
from mpc_real.franka_mpc_common import extract_parameters
from camera_librealsense import Camera
from franka_robot import FrankaRobot
import time
import sys
import numpy as np


class MPCPolicy:
    def __init__(self, path='mpc_real/'):
        model, solver, codeoptions = generate_pathplanner(create=False, path=path)
        self.model = model
        self.solver = solver
        self.codeoptions = codeoptions
        # robot set
        self.robot = FrankaRobot("192.168.5.12")
        self.gripper = self.robot.gripper
        # real env set
        self.offset = np.array([0.8, 0.75])  # robot base relative to the origin in simulator
        self.goal = np.array([0.45, 0])
        self.subgoal = self.goal
        self.init = np.array([0.45, 0.35])
        self.dt = 0.5  # time interval in real env
        self.length = 40  # number of steps to take
        # obstacle set
        self.obst_size = np.array([0.015, 0.017]) #  (x/2, y/2)
        self.vel = 0.02  # velocity of obstacle
        self.obst_rel_robot = np.array([0.435, 0.153])  # middle pose relative to robot base
        self.pos_dif = 0.15
        self.center_x = 0.435 + self.offset[0]
        # camera set
        self.camera = Camera()


    def initialize(self):
        # start camera
        self.camera.start()
        # move robot to initial pose
        self.robot.move([0, 0, 0.05])
        pose = np.append(self.init, 0.07)
        self.robot.move_to_init(pose)
        input("Enter to lower the gripper.")
        disp_z = pose[2] - 0.02
        self.robot.move([0, 0, -disp_z])
        input("Enter to grasp the object.")
        self.robot.clamp()
        self.robot.move([0, 0, disp_z])
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
            dist, _ = self.camera.get_distance(frame, add_to_frame=False)
        except IndexError:
            dist = 0.041
        dist -= 0.041  # relative to origin
        return dist

    def get_obs_pose(self, dist):
        # get obstacle pose
        dyn_obstacles = np.append(self.obst_rel_robot+self.offset, self.obst_size)
        dyn_obstacles[0] += dist - self.pos_dif
        return np.array([dyn_obstacles])

    def finish(self):
        self.camera.stop()
        cur_pose = self.robot.current_pose()
        disp_z = cur_pose[2] - 0.02
        self.robot.move([0, 0, -disp_z])
        self.robot.release()

    def play(self):
        pre_dist = None
        sign = 1
        xinit = self.init + self.offset
        for i in range(self.length):
            print("xinit: ", xinit)
            t1 = time.time()
            dist = self.get_obs_distance()
            if pre_dist:
                sign = np.sign(dist - pre_dist)
            pre_dist = dist
            dyn_obstacles = self.get_obs_pose(dist)
            goal = self.goal + self.offset
            subgoal = self.subgoal + self.offset
            parameters = extract_parameters(subgoal, goal, self.dt, self.model.N, dyn_obstacles, sign*self.vel, self.pos_dif,
                                            self.center_x)

            problem = self.get_problem(xinit, parameters)

            # call the solver
            output, exitflag, info = self.solver.solve(problem)
            assert exitflag >= 0, "Problem happended in solver."
            sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
                             .format(info.it, info.solvetime))

            # extract output of solver
            temp = np.zeros((self.model.nvar, self.model.N))
            for i in range(0, self.model.N):
                temp[:, i] = output['x{0:02d}'.format(i + 1)]
            pred_u = temp[0:2, :]
            pred_x = temp[3:5, :]

            real_action = pred_u[:, 0]
            self.robot.move([*real_action, 0])
            print("action:", real_action)
            # update the original pose
            cur_pose = self.robot.current_pose()
            x_init = cur_pose[0] + self.offset[0]
            y_init = cur_pose[1] + self.offset[1]
            xinit = np.array([x_init, y_init])
            print("time:", time.time() - t1)
            time.sleep(self.dt - (time.time() - t1))
        self.finish()









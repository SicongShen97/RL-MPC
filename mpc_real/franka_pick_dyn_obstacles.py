import time
import sys
sys.path.append('..')
import casadi
import matplotlib.pyplot as plt
import numpy as np
import forcespro
import forcespro.nlp
from mpc_real.franka_mpc_common import *
from camera_librealsense import Camera
from mpc_real.franka_plot import MPCDebugPlot
sys.path.append('../')
from common_real import get_args


def discrete_dynamics(z):
    # z = [dx, dy, dz, s, x, y, z]
    # x = [x, y, z], u = [dx, dy, dz]
    u = z[0:3]
    x = z[4:7]
    return casadi.vertcat(x[0] + u[0],
                          x[1] + u[1],
                          x[2])


def objective(z, p):
    # z = [dx, dy, dz, s, x, y, z]
    # p = [sub_goal_x, sub_goal_y, sub_goal_z, goal_x, goal_y, goal_z,
    # obst1_x, obst1_y, obst1_z, obst1_l, obst1_w, obst1_h,
    # obst2_x, obst2_y, obst2_z, obst2_l, obst2_w, obst2_h]
    u = z[0:3]
    x = z[4:7]
    goal = p[0:3]
    return ((x[0] - goal[0])**2 + (x[1] - goal[1])**2) + (u[0]**2 + u[1]**2)*10 + 100*z[3]**2
    # return ((x[0] - goal[0])**2 + (x[1] - goal[1])**2)*0.1 + 200*z[3]**2

def objectiveN(z, p):
    # z = [dx, dy, dz, s, x, y, z]
    # p = [sub_goal_x, sub_goal_y, sub_goal_z, goal_x, goal_y, goal_z,
    # obst1_x, obst1_y, obst1_z, obst1_l, obst1_w, obst1_h,
    # obst2_x, obst2_y, obst2_z, obst2_l, obst2_w, obst2_h]
    u = z[0:3]
    return 10*(u[0]**2 + u[1]**2) + 100*z[3]**2


def S(a, x1, x2):
    t = x1 * casadi.exp(x1 * a) + x2 * casadi.exp(x2 * a)
    b = casadi.exp(x1 * a) + casadi.exp(x2 * a)
    return t / b


def inequality_constraints(z, p):
    # z = [dx, dy, dz, s, x, y, z]
    # p = [sub_goal_x, sub_goal_y, sub_goal_z, goal_x, goal_y, goal_z,
    # obst1_x, obst1_y, obst1_z, obst1_l, obst1_w, obst1_h,
    # obst2_x, obst2_y, obst2_z, obst2_l, obst2_w, obst2_h]
    x = z[4:7]

    p_x = x[0]
    p_y = x[1]

    x_o1 = p[6]
    y_o1 = p[7]
    x_o2 = p[12]
    y_o2 = p[13]

    grip_w_x = 0.015
    grip_w_y = 0.038

    dx_o1 = p[9] + grip_w_x
    dy_o1 = p[10] + grip_w_y
    xp1 = casadi.fabs(p_x - x_o1) / dx_o1
    yp1 = casadi.fabs(p_y - y_o1) / dy_o1

    return casadi.vertcat(
        S(6, xp1, yp1) + z[3],  # obstacle1
        # casadi.sqrt((p_x - x_o1) ** 2 + (p_y - y_o1) ** 2) + z[3],
        casadi.sqrt((p_x - x_o2) ** 2 + (p_y - y_o2) ** 2) + z[3],  # obstacle2(square)
    )


def generate_pathplanner(create=True, path=''):
    """
    Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function.
    """
    # Model Definitions
    # ----------
    # dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 10
    # z = [dx, dy, dz, s, x, y, z]
    model.nvar = 7  # number of variables
    model.neq = 3  # number of equality constraints
    model.nh = 2    # number of nonlinear inequality constraints
    model.npar = 6 + 6 * 2  # number of runtime parameters

    model.objective = objective
    model.objectiveN = objectiveN

    # equalities
    model.eq = discrete_dynamics
    model.E = np.concatenate([np.zeros((3, 4)), np.eye(3)], axis=1)

    # initial state
    model.xinitidx = [4, 5, 6]

    # inequality
    model.ineq = inequality_constraints

    model.hu = np.array([+np.inf, +np.inf])
    model.hl = np.array([1.0, 0.025 + 0.04])

    # Inequality constraints
    #                   [ dx,   dy,    dz,     s,        x,       y,     z]
    model.lb = np.array([-0.01, -0.01, -0.01,  0,        -np.inf, +0.4,  -np.inf])
    model.ub = np.array([+0.01, +0.01, +0.01,  +np.inf,  +np.inf, +1.2,  +np.inf])

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FrankaFORCESNLPsolver_dyn_obsts')
    codeoptions.printlevel = 0
    codeoptions.overwrite = 1

    if create:
        solver = model.generate_solver(options=codeoptions)
    else:
        solver = forcespro.nlp.Solver.from_directory(path + "FrankaFORCESNLPsolver_dyn_obsts")

    return model, solver, codeoptions


def main():
    # generate code for estimator
    model, solver, codeoptions = generate_pathplanner(create=True)
    args = get_args()
    camera = Camera()
    camera.start()
    # Simulation
    # ----------
    # Variables for storing simulation data
    goal = np.array([0.0 + 0.5 + 0.8, -0.3 + 0.75, 0.4])  # relative to robot base [0.5, -0.3]
    t = 0
    dt = 0.5   # real env set
    vels = np.array([0.02, 0.03])   # real env set
    pos_dif = 0.1  # real env set
    center_x = 0.5 + 0.8  # real env set
    # dyn pos from camera
    frame_init = camera.get_frame()
    dists, _ = camera.get_distance(frame_init, add_to_frame=False)
    offsets = np.array([0.041, 0.04])
    dists -= offsets  # relative to origin
    dyn_obstacles = np.array([[dists[0] - pos_dif + 0.5 + 0.8,  0.1 + 0.75, 0.4, 0.045, 0.017, 0.015],
                              [dists[1] - pos_dif + 0.5 + 0.8, -0.1 + 0.75, 0.4, 0.015, 0.017, 0.015]])
    sim_length = 180

    # Set runtime parameters
    # ----------
    # Set initial state value
    xinit = np.array([0.5 + 0.8, 0.3 + 0.75, 0.4])   # relative to robot base [0.5, 0.3]
    x0i = np.zeros(7)
    x0i[4:7] = xinit
    x0 = np.reshape(x0i, (7, 1))
    pred = np.repeat(x0, model.N, axis=1)  # first prediction corresponds to initial guess
    parameters = extract_parameters(goal, goal, dt, model.N, dyn_obstacles, vels, pos_dif, center_x)
    obs = make_obs(parameters[0])  # simulate real obstacle positions

    problem = {"x0": pred,
               "xinit": xinit}

    debug_plot = MPCDebugPlot(args, sim_length=sim_length, model=model)
    debug_plot.createPlot(xinit=xinit, pred_x=pred[4:7], pred_u=pred[0:3], k=0, parameters=parameters, obs=obs)

    sim_timestep = 0
    pre_dists = np.array([None, None])
    signs = np.array([1, 1])
    # input("Start to move obstacles.")
    for k in range(sim_length):
        t1 = time.time()
        # if k % 10 == 9:
        #     signs *= -1
        # dyn pos from camera
        frame = camera.get_frame()
        dists, _ = camera.get_distance(frame, add_to_frame=False)
        dists -= offsets  # real env set
        if pre_dists.any():
            signs = np.sign(dists - pre_dists)
        pre_dists = dists
        dyn_obstacles = np.array([[dists[0] - pos_dif + 0.5 + 0.8, 0.1 + 0.75, 0.4, 0.045, 0.017, 0.015],
                                  [dists[1] - pos_dif + 0.5 + 0.8, -0.1 + 0.75, 0.4, 0.015, 0.017, 0.015]])
        # dyn_obstacles = np.array(obs["real_obstacle_info"])
        # move initial position to solve problem further
        problem["xinit"] = xinit

        parameters = extract_parameters(goal, goal, dt, model.N, dyn_obstacles, vels*signs, pos_dif, center_x)
        print(parameters)
        print('*'*20)
        problem["all_parameters"] = np.reshape(parameters, (model.npar * model.N, 1))

        # call the solver
        output, exitflag, info = solver.solve(problem)
        assert exitflag >= 0, print("problem, ", exitflag, "info:", info, "output", output)
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"
                         .format(info.it, info.solvetime))

        # Plot results
        # ------------
        # extract output of solver
        temp = np.zeros((model.nvar, model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i + 1)]
        pred_u = temp[0:3, :]
        pred_x = temp[4:7, :]
        xinit = pred_x[:, 1]  # take direct the next state
        obs = make_obs(parameters[1])  # simulate real obstacle positions
        debug_plot.updatePlots(xinit, goal, pred_x, pred_u, sim_timestep, parameters, obs)

        if k == sim_length - 1:
            debug_plot.show()
        else:
            debug_plot.draw()

        # change problem statement
        t += dt
        sim_timestep = sim_timestep + 1
        if dt - (time.time() - t1) > 0:
            plt.pause(dt - (time.time() - t1))
        print("time", time.time() - t1)

if __name__ == '__main__':
    main()


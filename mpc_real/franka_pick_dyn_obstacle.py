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


def discrete_dynamics(z):
    # z = [dx, dy, s, x, y]
    # x = [x, y], u = [dx, dy]
    u = z[0:2]
    x = z[3:5]
    return casadi.vertcat(x[0] + u[0],
                          x[1] + u[1])


def objective(z, p):
    # z = [dx, dy, s, x, y]
    # p = [sub_goal_x, sub_goal_y, goal_x, goal_y, obst_x, obst_y, obst_l, obst_w]
    u = z[0:2]
    x = z[3:5]
    goal = p[0:2]
    return ((x[0] - goal[0])**2 + (x[1] - goal[1])**2) + (u[0]**2 + u[1]**2)*10 + 100*z[2]**2


def objectiveN(z, p):
    # z = [dx, dy, x, y, s]
    # p = [sub_goal_x, sub_goal_y, goal_x, goal_y, obst_x, obst_y, obst_l, obst_w]
    u = z[0:2]
    return 10*(u[0]**2 + u[1]**2) + 100*z[2]**2


def inequality_constraints(z, p):
    # z = [dx, dy, s, x, y]
    # p = [sub_goal_x, sub_goal_y, goal_x, goal_y, obst_x, obst_y, obst_l, obst_w]
    u = z[0:2]
    x = z[3:5]

    p_x = x[0]
    p_y = x[1]

    x_o = p[4]
    y_o = p[5]

    return casadi.vertcat(
        casadi.sqrt((p_x - x_o) ** 2 + (p_y - y_o) ** 2) + z[2]
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
    # z = [dx, dy, s, x, y]
    model.nvar = 5  # number of variables
    model.neq = 2  # number of equality constraints
    model.nh = 1    # number of nonlinear inequality constraints
    model.npar = 4 + 4  # number of runtime parameters

    model.objective = objective
    model.objectiveN = objectiveN

    # equalities
    model.eq = discrete_dynamics
    model.E = np.concatenate([np.zeros((2, 3)), np.eye(2)], axis=1)

    # initial state
    model.xinitidx = [3, 4]

    # inequality
    model.ineq = inequality_constraints

    model.hu = np.array([+np.inf])
    model.hl = np.array([0.025 + 0.035])

    # Inequality constraints
    #                   [ dx,   dy,    s,       x,       y]
    model.lb = np.array([-0.01, -0.01, 0,       -np.inf, +0.4])
    model.ub = np.array([+0.01, +0.01, +np.inf, +np.inf, +1.2])

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FrankaFORCESNLPsolver_10mm')
    codeoptions.printlevel = 0
    codeoptions.overwrite = 1

    if create:
        solver = model.generate_solver(options=codeoptions)
    else:
        solver = forcespro.nlp.Solver.from_directory(path + "FrankaFORCESNLPsolver_10mm")

    return model, solver, codeoptions


def main():
    # generate code for estimator
    model, solver, codeoptions = generate_pathplanner(create=False)

    camera = Camera()
    camera.start()
    # Simulation
    # ----------
    # Variables for storing simulation data
    goal = np.array([0.45 + 0.8, -0.0 + 0.75])  # relative to robot base [0.45, -0.0]
    t = 0
    dt = 0.5   # real env set
    vel = 0.02   # real env set
    pos_dif = 0.15  # real env set
    center_x = 0.435 + 0.8  # real env set
    # dyn pos from camera
    frame_init = camera.get_frame()
    dist, _ = camera.get_distance(frame_init, add_to_frame=False)
    dist -= 0.041  # relative to origin
    dyn_obstacles = np.array([[dist - pos_dif + 0.435 + 0.8, 0.153 + 0.75, 0.015, 0.017]])   # pose relative to robot base[0.435, 0.153]

    sim_length = 180

    # Set runtime parameters
    # ----------
    # Set initial state value
    xinit = np.array([0.45 + 0.8, 0.35 + 0.75])   # relative to robot base [0.45, 0.35]
    x0i = np.zeros(5)
    x0i[3:5] = xinit
    x0 = np.reshape(x0i, (5, 1))
    pred = np.repeat(x0, model.N, axis=1)  # first prediction corresponds to initial guess
    parameters = extract_parameters(goal, goal, dt, model.N, dyn_obstacles, vel, pos_dif, center_x)
    obs = make_obs(parameters[0])  # simulate real obstacle positions

    problem = {"x0": pred,
               "xinit": xinit}

    debug_plot = MPCDebugPlot(sim_length=sim_length, model=model)
    debug_plot.createPlot(xinit=xinit, pred_x=pred[3:5], pred_u=pred[0:2], k=0, parameters=parameters, obs=obs)

    sim_timestep = 0
    pre_dist = None
    sign = 1
    for k in range(sim_length):
        t1 = time.time()
        # dyn pos from camera
        frame = camera.get_frame()
        dist, _ = camera.get_distance(frame, add_to_frame=False)
        dist -= 0.041  # real env set
        if pre_dist:
            sign = np.sign(dist - pre_dist)
        pre_dist = dist
        dyn_obstacles = np.array([[dist - pos_dif + 0.435 + 0.8, 0.153 + 0.75, 0.015, 0.017]])
        # move initial position to solve problem further
        problem["xinit"] = xinit

        parameters = extract_parameters(goal, goal, dt, model.N, dyn_obstacles, vel*sign, pos_dif, center_x)
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
        pred_u = temp[0:2, :]
        pred_x = temp[3:5, :]
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

        plt.pause(dt - (time.time() - t1))
        print("time", time.time() - t1)

if __name__ == '__main__':
    main()


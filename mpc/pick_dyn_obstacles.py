# continuous time dynamics
# control action - acceleration
# 2 goals. Main + Sub goal
# performs one step
# relaxing variable
# 3D
import argparse
import sys
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
import casadi
import time
import math

import sys
sys.path.append("../")
from mpc.mpc_common import extract_parameters, make_obs, get_args


def continuous_dynamics(x, u, p):
    # calculate dx/dt
    return casadi.vertcat(x[3],
                          x[4],
                          0,
                          u[0],
                          u[1],
                          0
                          )


def objective(z, p):
    u = z[0:4]
    x = z[4:10]
    acc_x_rel = u[0] / 30.0
    acc_y_rel = u[1] / 30.0
    acc_z_rel = u[2] / 30.0

    return 10 * ((x[0] - p[0]) ** 2 + (x[1] - p[1]) ** 2) + \
           0.01 * acc_x_rel ** 2 + 0.01 * acc_y_rel ** 2 + 700 * u[3] ** 2


def objectiveN(z, p):
    u = z[0:4]
    x = z[4:10]
    vel_x_rel = x[3]
    vel_y_rel = x[4]
    vel_z_rel = x[5]
    return objective(z, p) + 0.1 * vel_x_rel ** 2 + 0.1 * vel_y_rel ** 2

def S(a, x1, x2):
    t = x1 * casadi.exp(x1 * a) + x2 * casadi.exp(x2 * a)
    b = casadi.exp(x1 * a) + casadi.exp(x2 * a)
    return t / b

def inequality_constraints(z, p):
    u = z[0:4]
    x = z[4:10]

    p_x = x[0]
    p_y = x[1]
    p_z = x[2]

    x_o1 = p[6]
    y_o1 = p[7]
    z_o1 = p[8]

    x_o2 = p[12]
    y_o2 = p[13]
    z_o2 = p[14]

    grip_w_x = 0.03
    grip_w_y = 0.048

    dx_o2 = p[15] + grip_w_x
    dy_o2 = p[16] + grip_w_y
    xp2 = casadi.fabs(p_x - x_o2) / dx_o2
    yp2 = casadi.fabs(p_y - y_o2) / dy_o2

    return casadi.vertcat(
        casadi.sqrt((p_x - x_o1) ** 2 + (p_y - y_o1) ** 2) + u[3],   # obstacle 1 (square)
        S(6, xp2, yp2) + u[3],     # obstacle 2,
        p_x + u[3]
    )


def generate_pathplanner(create=True, path=''):
    """
    Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function.
    """
    # Model Definition
    # ----------------

    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 8  # horizon length
    model.nvar = 10  # number of variables
    model.neq = 6  # number of equality constraints
    model.nh = 3
    model.npar = 6 + 6 * 2  # number of runtime parameters

    model.objective = objective
    model.objectiveN = objectiveN


    model.continuous_dynamics = continuous_dynamics

    model.E = np.concatenate([np.zeros((6, 4)), np.eye(6)], axis=1)

    grip_w = 0 #0.045

    # Inequality constraints
    #                   [ ax,     ay,    az,   relax         xPos,         yPos,          zPos    xVel, yVel, zVel]
    model.lb = np.array([-30.0, -30.0, -30.0, 0,           -np.inf,    +0.4 + grip_w,   +0.4,     -1.0, -1.0, -1.0])
    model.ub = np.array([+30.0, +30.0, +30.0, +np.inf,     +np.inf,    +1.1 - grip_w,   +0.5,    +1.0, +1.0, +1.0])

    # General (differentiable) nonlinear inequalities hl <= h(z,p) <= hu
    model.ineq = inequality_constraints

    # Upper/lower bounds for inequalities
    model.hu = np.array([+np.inf,     +np.inf, +np.inf])
    model.hl = np.array([0.05 + 0.05, 1,       0])

    # Initial condition on vehicle states x
    model.xinitidx = range(4, 10)  # use this to specify on which variables initial conditions
    # are imposed

    # Solver generation
    # -----------------

    # Set solver options
    codeoptions = forcespro.CodeOptions('FORCESNLPsolver2')
    codeoptions.maxit = 200  # Maximum number of iterations
    codeoptions.printlevel = 0
    codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size,
    #                             2 optimize for speed, 3 optimize for size & speed
    # codeoptions.nlp.bfgs_init = 3.0 * np.identity(4)  # initialization of the hessian
    #                             approximation
    codeoptions.noVariableElimination = 1.
    codeoptions.overwrite = 1
    codeoptions.nlp.integrator.Ts = 0.01
    codeoptions.nlp.integrator.nodes = 5
    codeoptions.nlp.integrator.type = 'ERK4'
    # codeoptions.nlp.TolEq = 1E-2
    # codeoptions.nlp.TolIneq = 1E-2
    # codeoptions.nlp.TolComp = 1E-2
    # codeoptions.nlp.TolStat = 1E-2
    # change this to your server or leave uncommented for using the
    # standard embotech server at https://forces.embotech.com
    # codeoptions.server = 'https://forces.embotech.com'

    # Creates code for symbolic model formulation given above, then contacts
    # server to generate new solver
    if create:
        solver = model.generate_solver(options=codeoptions)
    else:
        solver = forcespro.nlp.Solver.from_directory(path + "FORCESNLPsolver2")

    return model, solver, codeoptions


def main():
    from mpc.plot import MPCDebugPlot
    args = get_args()
    args.env = 'FetchPickDynObstaclesEnv-v2'
    # generate code for estimator
    model, solver, codeoptions = generate_pathplanner(create=args.mpc_gen)

    # Simulation
    # ----------
    # Variables for storing simulation data
    goal = np.array([1.45, 0.43, 0.469])
    t = 0
    dt = codeoptions.nlp.integrator.Ts
    vels = np.array([0.5, 0.00001])
    shifts = np.array([0.0, 0.0])
    pos_difs = np.array([0.22, 0.13])

    stat_obstacles = []
    dyn_obstacles = [[1.3, 0.60, 0.435, 0.03, 0.03, 0.03], [1.28, 0.80, 0.435, 0.12, 0.03, 0.03]]

    sim_length = 60

    # Set runtime parameters

    # Set initial guess to start solver from
    xinit = np.array([1.3, 1.0, 0.44, 0, 0, 0])

    # print(obst_pos, obst_vel)
    # exit()
    debug_plot = MPCDebugPlot(args, sim_length=sim_length, model=model)

    # plot inputs and states and make window interactive
    # generate plot with initial values
    x0i = np.zeros(10)
    x0i[4:10] = xinit
    x0 = np.reshape(x0i, (10, 1))
    pred = np.repeat(x0, model.N, axis=1)  # first prediction corresponds to initial guess
    parameters = extract_parameters(goal, goal, t, dt, model.N, dyn_obstacles, vels, shifts, pos_difs, stat_obstacles)

    obs = make_obs(parameters[0])  # simulate real obstacle positions
    debug_plot.createPlot(xinit=xinit, pred_x=pred[4:10], pred_u=pred[0:4], k=0, parameters=parameters, obs=obs)

    done = False

    problem = {"x0": pred,
               "xinit": xinit}

    # while not done:
    sim_timestep = 0
    for k in range(sim_length):
        # Set initial condition
        parameters = extract_parameters(goal, goal, t, dt, model.N, dyn_obstacles, vels, shifts, pos_difs, stat_obstacles)
        problem["all_parameters"] = np.reshape(parameters, (model.npar * model.N, 1))

        print('Solve ', k, problem["xinit"])
        print(parameters[0])

        output, exitflag, info = solver.solve(problem)
        if exitflag != 1:
            plt.show()
            print('ERROR')
            exit()

        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n" \
                         .format(info.it, info.solvetime))

        # Plot results
        # ------------
        # extract output of solver
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:d}'.format(i + 1)]
        pred_u = temp[0:4, :]
        pred_x = temp[4:10, :]

        # Apply optimized input u of first stage to system and save simulation data
        # z = np.concatenate((pred_u[:, 0], pred_x[:, 0]))
        # p = np.array(problem['all_parameters'][0:model.npar])
        # c, jacc = solver.dynamics(z, p)
        # next_x = np.reshape(c, newshape=(4))
        next_x = pred_x[:, 1]  # take direct calculated state

        # print('Cost:', model.objective(z, p))

        # plot results of current simulation step
        obs = make_obs(parameters[0])  # simulate real obstacle positions
        debug_plot.updatePlots(next_x, next_x, pred_x, pred_u, sim_timestep, parameters, obs, None)

        # change problem statement
        t += dt
        sim_timestep = sim_timestep + 1
        # move initial position to solve problem further
        problem["xinit"] = next_x

        if k == sim_length - 1:
            debug_plot.show()
        else:
            print('draw')
            debug_plot.draw()
        done = True
        #input('check')

    # time.sleep(2.4)
    # update_plots()


if __name__ == "__main__":
    main()

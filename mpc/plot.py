# this class supports the subgoal parameter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
import seaborn as sns; sns.set()

def rectangle(bbox, ax=None, *args, **kwargs):
    """Draw an ellipse filling the given bounding box."""
    if ax == None:
        ax = plt.gca()
    x, y, w, h = bbox
    shape = matplotlib.patches.Rectangle((x - w / 2, y - h / 2), w, h, *args, **kwargs)
    ax.add_artist(shape)
    return ax

def ellipse(bbox, ax=None, *args, **kwargs):
    """Draw an ellipse filling the given bounding box."""
    if ax == None:
        ax = plt.gca()
    x, y, w, h = bbox
    shape = matplotlib.patches.Ellipse((x, y), w, h, *args, **kwargs)
    ax.add_artist(shape)
    return ax

class MPCDebugPlot:

    grip_w_x = None
    grip_w_y = None
    grip_w_z = None

    obstacle_color = []

    safe_areas = {
        'FetchPickDynSqrObstacle-v1': [[0.05, 0.05], [0.05, 0.05], [0.05, 0.05]],
        'FetchPickDynObstaclesEnv-v1': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FetchPickDynObstaclesEnv-v2': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FetchPickDynLiftedObstaclesEnv-v1': [[0.05, 0.055 + 0.02, 0.055], [0.05, 0.05 + 0.02, 0.05], [0.05, 0.02, 0.02]],
        'FetchPickDynObstaclesMaxEnv-v1': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FrankaFetchPickDynSqrObstacle-v1': [[0.05, 0.05], [0.05, 0.05], [0.05, 0.05]],
    }

    obstacle_colors = {
        'FetchPickDynSqrObstacle-v1': ['#416ab6', '#416ab6'],
        'FetchPickDynObstaclesEnv-v1': ['#416ab6', '#5aa9a2'],
        'FetchPickDynObstaclesEnv-v2': ['#416ab6', '#416ab6'],
        'FetchPickDynLiftedObstaclesEnv-v1': ['#416ab6', '#416ab6', '#5aa9a2'],
        'FetchPickDynObstaclesMaxEnv-v1': ['#416ab6', '#5aa9a2'],

        'FrankaFetchPickDynSqrObstacle-v1': ['#416ab6', '#416ab6'],
    }

    def __init__(self, args, sim_length: int, model):
        self.x = np.zeros((6, sim_length + 1))  # states
        self.u = np.zeros((4, sim_length))  # inputs
        self.err = np.zeros(sim_length)  # errors
        self.sim_length = sim_length
        self.model = model
        self.setup(args.env)

    def setup(self, env):
        self.grip_w_x = list(reversed(self.safe_areas[env][0]))
        self.grip_w_y = list(reversed(self.safe_areas[env][1]))
        self.grip_w_z = list(reversed(self.safe_areas[env][2]))
        self.obstacle_color = list(reversed(self.obstacle_colors[env]))     # reverse for z-index of the layers

    def show(self):
        plt.show()

    def draw(self):
        plt.draw()

    def createPlot(self, xinit, pred_x, pred_u, k, parameters, obs):
        """Creates a plot and adds the initial data provided by the arguments"""
        x = self.x
        u = self.u
        model = self.model
        sim_length = self.sim_length
        err = self.err

        x[:, k] = xinit
        u[:, k] = pred_u[:, 0]

        # Create empty plot
        fig = plt.figure()
        fig.set_size_inches(17, 9, forward=True)
        plt.clf()
        gs = GridSpec(5, 3, figure=fig)

        # Plot trajectory
        ax_pos = fig.add_subplot(gs[:, 0])
        # l0, = ax_pos.plot(np.transpose(path_points[0, :]), np.transpose(path_points[1, :]), 'rx')
        l11, = ax_pos.plot(parameters[0][3], parameters[0][4], 'bo') # main goal
        l1, = ax_pos.plot(parameters[0][0], parameters[0][1], 'kx') # subgoal
        plt.title('Position', fontsize=16)
        # plt.axis('equal')
        plt.xlim([1.05, 1.55])
        plt.ylim([0.4, 1.1])
        plt.xlabel('x-coordinate', fontsize=16)
        plt.ylabel('y-coordinate', fontsize=16)
        l2, = ax_pos.plot(x[0, 0], x[1, 0], 'b-')
        l3, = ax_pos.plot(pred_x[0, :], pred_x[1, :], 'g-')
        l4, = ax_pos.plot(xinit[0], xinit[1], 'bx')
        l5, = ax_pos.plot(xinit[0], xinit[1], 'g+')
        l6, = ax_pos.plot(xinit[0], xinit[1], 'b+')
        l7, = ax_pos.plot(xinit[0], xinit[1], 'r+')
        # ax_pos.legend([l1, l11, l2, l3, l4, l5, l6], ['subgoal', 'goal', 'robot trajectory', \
        #                                  'predicted robot traj.', 'init pos', 'next pos', 'next actual pos', 'target'], loc='lower right')

        obst_sz = model.hl[0]
        N = parameters.shape[0]

        # Draw obstacles

        real_obstacles = list(obs['real_obstacle_info'][::-1])

        safe_areas = []
        for i in range(len(real_obstacles)):
            obst = real_obstacles[i].copy()
            obst[3] += self.grip_w_x[i]
            obst[4] += self.grip_w_y[i]
            obst[5] += self.grip_w_z[i]
            safe_areas.append(obst)

        for i in range(len(real_obstacles)):
            obst = real_obstacles[i]
            bbox = (obst[0], obst[1], obst[3] * 2, obst[4] * 2)
            rectangle(bbox, fill=True, linestyle=":", edgecolor='red', color=self.obstacle_color[i])

        for i in range(len(safe_areas)):
            obst = safe_areas[i]
            bbox = (obst[0], obst[1], obst[3] * 2, obst[4] * 2)
            rectangle(bbox, fill=False, linestyle=":", edgecolor='red')

        # place grip ellipse
        bbox = (xinit[0], xinit[1], self.grip_w_y[0] * 2, self.grip_w_y[0] * 2)
        ellipse(bbox, fill=False, linestyle=":", edgecolor='blue')

        for i in range(N):
            p = parameters[i]
            bbox = (p[6], p[7], obst_sz, obst_sz)
            ellipse(bbox, fill=False, linestyle=":", edgecolor='blue' if i == 0 else 'green',
                    alpha=(1.0 - i / N))
            bbox = (p[12], p[13], obst_sz, obst_sz)
            ellipse(bbox, fill=False, linestyle=":", edgecolor='blue' if i == 0 else 'green',
                    alpha=(1.0 - i / N))
            # bbox = (p[18], p[19], obst_sz, obst_sz)
            # ellipse(bbox, fill=False, linestyle=":", edgecolor='blue' if i == 0 else 'green',
            #         alpha=(1.0 - i / N))

        # Plot action X
        ax_dx = fig.add_subplot(5, 3, 2)
        plt.grid("both")
        plt.title('Acceleration X', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[0], model.ub[0]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[0], model.lb[0]]), 'r:')
        ax_dx.step(range(0, k + 1), u[0, 0:k + 1], 'b-')
        ax_dx.step(range(k, k + model.N), pred_u[0, :], 'g-')

        # Plot action Y
        ax_dy = fig.add_subplot(5, 3, 5)
        plt.grid("both")
        plt.title('Acceleration Y', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[1], model.ub[1]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[1], model.lb[1]]), 'r:')
        ax_dy.step(range(0, k + 1), u[1, 0:k + 1], 'b-')
        ax_dy.step(range(k, k + model.N), pred_u[1, :], 'g-')

        # Plot action Z
        ax_dz = fig.add_subplot(5, 3, 8)
        plt.grid("both")
        plt.title('Acceleration Z', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[2], model.ub[2]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[2], model.lb[2]]), 'r:')
        ax_dz.step(range(0, k + 1), u[2, 0:k + 1], 'b-')
        ax_dz.step(range(k, k + model.N), pred_u[2, :], 'g-')

        # Plot velocity
        ax_velX = fig.add_subplot(5, 3, 3)
        plt.grid("both")
        plt.title('Velocity X', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[7], model.ub[7]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[7], model.lb[7]]), 'r:')
        ax_velX.plot(range(0, k + 1), x[3, 0:k + 1], '-b')
        ax_velX.plot(range(k + 1, k + model.N + 1), pred_x[3, :], 'g-')

        # Plot velocity
        ax_velY = fig.add_subplot(5, 3, 6)
        plt.grid("both")
        plt.title('Velocity Y', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[8], model.ub[8]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[8], model.lb[8]]), 'r:')
        ax_velY.plot(range(0, k + 1), x[4, 0:k + 1], '-b')
        ax_velY.plot(range(k + 1, k + model.N + 1), pred_x[4, :], 'g-')

        # Plot velocity
        ax_velZ = fig.add_subplot(5, 3, 9)
        plt.grid("both")
        plt.title('Velocity Z', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[9], model.ub[9]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[9], model.lb[9]]), 'r:')
        ax_velZ.plot(range(0, k + 1), x[5, 0:k + 1], '-b')
        ax_velZ.plot(range(k + 1, k + model.N + 1), pred_x[5, :], 'g-')

        # Plot Z
        ax_Z = fig.add_subplot(5, 3, 14)
        plt.grid("both")
        plt.title('Position Z', fontsize=16)
        plt.xlim([0., sim_length - 1])
        # plt.plot([0, sim_length - 1], np.transpose([model.hu[4], model.hu[4]]), 'r:')
        # plt.plot([0, sim_length - 1], np.transpose([model.hl[4], model.hl[4]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([0.465, 0.465]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([0.405, 0.405]), 'r:')
        ax_Z.plot(range(0, k + 1), x[2, 0:k + 1], '-b')
        ax_Z.plot(range(k + 1, k + model.N + 1), pred_x[2, :], 'g-')

        # Plot relax var
        ax_Rel = fig.add_subplot(5, 3, 11)
        plt.grid("both")
        plt.title('Relaxation', fontsize=16)
        plt.xlim([0., sim_length - 1])
        #plt.plot([0, sim_length - 1], np.transpose([model.ub[3], model.ub[3]]), 'r:')
        #plt.plot([0, sim_length - 1], np.transpose([model.lb[3], model.lb[3]]), 'r:')
        ax_Rel.step(range(0, k + 1), u[3, 0:k + 1], 'b-')
        ax_Rel.step(range(k, k + model.N), pred_u[3, :], 'g-')

        # Plot trajectory
        ax_posZ = fig.add_subplot(5, 3, (12,15))
        plt.plot(parameters[0][1], parameters[0][2], 'kx')  # subgoal
        plt.title('Position', fontsize=16)
        # plt.axis('equal')
        plt.xlim([0.4, 1.1])
        plt.ylim([0.4, 0.55])
        plt.xlabel('y-coordinate', fontsize=16)
        plt.ylabel('z-coordinate', fontsize=16)
        ax_posZ.plot(x[1, 0], x[2, 0], 'b-')
        ax_posZ.plot(pred_x[1, :], pred_x[2, :], 'g-')

        for i in range(len(real_obstacles)):
            obst = real_obstacles[i]
            bbox = (obst[1], obst[2], obst[4] * 2, obst[5] * 2)
            rectangle(bbox, fill=True, linestyle=":", edgecolor='red', color=self.obstacle_color[i])

        for i in range(len(safe_areas)):
            obst = safe_areas[i]
            bbox = (obst[1], obst[2], obst[4] * 2, obst[5] * 2)
            rectangle(bbox, fill=False, linestyle=":", edgecolor='red')

        plt.tight_layout()

        # Make plot fullscreen. Comment out if platform dependent errors occur.
        mng = plt.get_current_fig_manager()

    def updatePlots(self, next_x, target_x, pred_x, pred_u, k, parameters, ob, new_ob=None):
        """Deletes old data sets in the current plot and adds the new data sets
        given by the arguments x, u and predicted_z to the plot.
        x: matrix consisting of a set of state column vectors
        u: matrix consisting of a set of input column vectors
        pred_x: predictions for the next N state vectors
        pred_u: predictions for the next N input vectors
        model: model struct required for the code generation of FORCESPRO
        k: simulation step
        apply_N: number of steps to apply
        """
        x = self.x
        u = self.u
        err = self.err
        model = self.model

        print('k: ', k)
        x[:, k + 1] = next_x
        u[:, k] = pred_u[:, 0]
        err[k] = 0

        fig = plt.gcf()
        ax_list = fig.axes

        robot_pos = next_x
        if new_ob is not None:
            if 'observation' in new_ob:
                robot_pos = new_ob['observation'][0:2]
                next_pred_pos = next_x[0:2]
                err[k] = np.linalg.norm(robot_pos - next_pred_pos)
        params = parameters[0]

        # Move obstacles
        rectangles = list(filter(lambda obj: type(obj) is matplotlib.patches.Rectangle, ax_list[0].get_children()))
        real_obstacles = list(ob['real_obstacle_info'][::-1])

        safe_areas = []
        for i in range(len(real_obstacles)):
            obst = real_obstacles[i].copy()
            obst[3] += self.grip_w_x[i]
            obst[4] += self.grip_w_y[i]
            safe_areas.append(obst)

        real_obstacles = real_obstacles + safe_areas

        for i in range(len(real_obstacles)):
            obj = rectangles[i]
            obst = real_obstacles[i]
            obj.set_xy((obst[0] - obst[3], obst[1] - obst[4]))

        # Move predicted obstacles
        ellipses = list(filter(lambda obj: type(obj) is matplotlib.patches.Ellipse, ax_list[0].get_children()))
        obstacles = []

        obstacles.append([next_x[0], next_x[1]])

        for p in parameters:
            obstacles.append([p[6], p[7]])
            obstacles.append([p[12], p[13]])
            #obstacles.append([p[18] + 10, p[19] + 10])

        # print(obstacles)
        # print(parameters, len(obstacles), len(ellipses))

        # Draw original and predicted obstacles
        for z in zip(ellipses, obstacles):
            e, o = z
            _, yPos = e.get_center()
            e.center = o[0], o[1]

        # Delete old data in plot
        ax_list[0].get_lines().pop(-1).remove()  # remove old target position
        ax_list[0].get_lines().pop(-1).remove()  # remove old next actual position
        ax_list[0].get_lines().pop(-1).remove()  # remove old next predicted
        ax_list[0].get_lines().pop(-1).remove()  # remove old init position
        ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
        ax_list[0].get_lines().pop(-1).remove()  # remove old trajectory
        #ax_list[0].get_lines().pop(-1).remove()  # remove old subgoal

        ax_list[1].get_lines().pop(-1).remove()  # remove old prediction ax
        ax_list[1].get_lines().pop(-1).remove()  # remove old ax

        ax_list[2].get_lines().pop(-1).remove()  # remove old prediction ay
        ax_list[2].get_lines().pop(-1).remove()  # remove old ay

        ax_list[3].get_lines().pop(-1).remove()  # remove old prediction az
        ax_list[3].get_lines().pop(-1).remove()  # remove old az

        ax_list[4].get_lines().pop(-1).remove()  # remove old vx prediction
        ax_list[5].get_lines().pop(-1).remove()  # remove old vy prediction
        ax_list[6].get_lines().pop(-1).remove()  # remove old vz prediction

        ax_list[7].get_lines().pop(-1).remove()  # remove old Z
        ax_list[8].get_lines().pop(-1).remove()  # remove old relax

        ax_list[9].get_lines().pop(-1).remove()  # remove old prediction of trajectory
        ax_list[9].get_lines().pop(-1).remove()  # remove old trajectory

        # Update plot with current simulation data
        # ax_list[0].plot(params[0], params[1], 'kx')  # plot new subgoal position

        ax_list[0].plot(x[0, 0:k+2], x[1, 0:k+2], '-b')  # plot new trajectory
        ax_list[0].plot(pred_x[0, 1:], pred_x[1, 1:], 'g-')  # plot new prediction of trajectory

        ax_list[0].plot(pred_x[0, 0], pred_x[1, 0], 'mx')  # plot new init position
        ax_list[0].plot(next_x[0], next_x[1], 'g+')  # plot new next position
        ax_list[0].plot(robot_pos[0], robot_pos[1], 'b+')  # plot new next actual position
        ax_list[0].plot(target_x[0], target_x[1], 'r+')  # plot new target

        ax_list[1].step(range(0, k+1), u[0, 0:k+1], 'b-')  # plot new dx
        ax_list[1].step(range(k, k + model.N), pred_u[0, :], 'g-')  # plot new prediction of ax

        ax_list[2].step(range(0, k+1), u[1, 0:k+1], 'b-')  # plot new dy
        ax_list[2].step(range(k, k + model.N), pred_u[1, :], 'g-')  # plot new prediction of ay

        ax_list[3].step(range(0, k + 1), u[2, 0:k + 1], 'b-')  # plot new dz
        ax_list[3].step(range(k, k + model.N), pred_u[2, :], 'g-')  # plot new prediction of az

        ax_list[4].plot(x[3, 0:k + 2], 'b-')  # plot new velocity
        ax_list[4].plot(range(k + 1, k + model.N), pred_x[3, 1:], 'g-')  # plot new prediction of velocity

        ax_list[5].plot(x[4, 0:k + 2], 'b-')  # plot new velocity
        ax_list[5].plot(range(k + 1, k + model.N), pred_x[4, 1:], 'g-')  # plot new prediction of velocity

        ax_list[6].plot(x[5, 0:k + 2], 'b-')  # plot new velocity
        ax_list[6].plot(range(k + 1, k + model.N), pred_x[5, 1:], 'g-')  # plot new prediction of velocity

        ax_list[7].plot(x[2, 0:k + 2], 'b-')  # plot Z
        ax_list[7].plot(range(k + 1, k + model.N), pred_x[2, 1:], 'g-')  # plot new prediction of Z

        ax_list[8].step(range(0, k + 1), u[3, 0:k + 1], 'b-')  # plot new relax var
        ax_list[8].step(range(k, k + model.N), pred_u[3, :], 'g-')  # plot new relax var

        ax_list[9].plot(params[1], params[2], 'kx')  # plot new subgoal position
        ax_list[9].plot(x[1, 0:k + 2], x[2, 0:k + 2], '-b')  # plot new trajectory
        ax_list[9].plot(pred_x[1, 1:], pred_x[2, 1:], 'g-')  # plot new prediction of trajectory

        plt.pause(0.5)


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

class MPCDebugPlotSmall:
    grip_w_x = None
    grip_w_y = None
    grip_w_z = None

    safe_areas = {
        'FetchPickDynSqrObstacle-v1': [[0.05, 0.05], [0.05, 0.05, 0.0], [0.05, 0.05]],
        'FetchPickDynObstaclesEnv-v1': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FetchPickDynObstaclesEnv-v2': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FetchPickDynLiftedObstaclesEnv-v1': [[0.05, 0.055 + 0.02, 0.055], [0.05, 0.05 + 0.02, 0.05],
                                              [0.05, 0.02, 0.02]],
        'FetchPickDynObstaclesMaxEnv-v1': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
    }

    obstacle_colors = {
        'FetchPickDynSqrObstacle-v1': ['#416ab6', '#416ab6'],
        'FetchPickDynObstaclesEnv-v1': ['#416ab6', '#5aa9a2'],
        'FetchPickDynObstaclesEnv-v2': ['#416ab6', '#416ab6'],
        'FetchPickDynLiftedObstaclesEnv-v1': ['#416ab6', '#416ab6', '#5aa9a2'],
        'FetchPickDynObstaclesMaxEnv-v1': ['#416ab6', '#5aa9a2'],
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
        self.obstacle_color = list(reversed(self.obstacle_colors[env]))  # reverse for z-index of the layers

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
        fig.set_size_inches(6, 8, forward=True)
        plt.clf()
        #gs = GridSpec(5, 2, figure=fig)

        # Plot trajectory
        ax_pos = fig.add_subplot()
        # l0, = ax_pos.plot(np.transpose(path_points[0, :]), np.transpose(path_points[1, :]), 'rx')
        l11, = plt.plot(parameters[0][3], parameters[0][4], 'bo') # main goal
        l1, = plt.plot(0, 0, 'kx') # subgoal
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
        ax_pos.legend([l1, l11, l2, l3], ['subgoal', 'goal', 'robot trajectory', \
                                         'predicted trajectory'], loc='upper right')

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
            # obstacles.append([p[18] + 10, p[19] + 10])

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

        # Update plot with current simulation data
        ax_list[0].plot(params[0], params[1], 'kx')  # plot new subgoal position

        ax_list[0].plot(x[0, 0:k+2], x[1, 0:k+2], '-b')  # plot new trajectory
        ax_list[0].plot(pred_x[0, 1:], pred_x[1, 1:], 'g-')  # plot new prediction of trajectory

        ax_list[0].plot(pred_x[0, 0], pred_x[1, 0], 'mx')  # plot new init position
        ax_list[0].plot(next_x[0], next_x[1], 'g+')  # plot new next position
        ax_list[0].plot(robot_pos[0], robot_pos[1], 'b+')  # plot new next actual position
        ax_list[0].plot(target_x[0], target_x[1], 'r+')  # plot new target

        plt.pause(0.25)


import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec


def rectangle(bbox, ax=None, *args, **kwargs):
    """Draw a rectangle filling the given bounding box."""
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

    safe_areas = {
        'FrankaPickDynSqrObstacles-v1': [0.0, 0.005, 0],
        'FrankaPickDynObstacles-v1': [0.0, 0.005, 0],
        'FrankaPickDynLiftedObstacles-v1': [0.0, 0.014, 0],

        'FetchPickDynObstaclesEnv-v2': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FetchPickDynLiftedObstaclesEnv-v1': [[0.05, 0.055 + 0.02, 0.055], [0.05, 0.05 + 0.02, 0.05], [0.05, 0.02, 0.02]],
        'FetchPickDynObstaclesMaxEnv-v1': [[0.05, 0.03], [0.05, 0.048], [0.05, 0.05]],
        'FrankaFetchPickDynSqrObstacle-v1': [[0.05, 0.05], [0.05, 0.05], [0.05, 0.05]],
    }

    obstacle_colors = {
        'FrankaPickDynSqrObstacles-v1': ['#416ab6', '#416ab6'],
        'FrankaPickDynLiftedObstacles-v1': ['#416ab6', '#416ab6', '#5aa9a2'],

        'FetchPickDynObstaclesEnv-v1': ['#416ab6', '#5aa9a2'],
        'FetchPickDynObstaclesEnv-v2': ['#416ab6', '#416ab6'],
        'FetchPickDynLiftedObstaclesEnv-v1': ['#416ab6', '#416ab6', '#5aa9a2'],
        'FetchPickDynObstaclesMaxEnv-v1': ['#416ab6', '#5aa9a2'],
        'FrankaFetchPickDynSqrObstacle-v1': ['#416ab6', '#416ab6'],
    }

    def __init__(self, args, sim_length: int, model):
        self.x = np.zeros((3, sim_length + 1))  # states
        self.u = np.zeros((3, sim_length))  # inputs
        self.err = np.zeros(sim_length)  # errors
        self.sim_length = sim_length
        self.model = model
        self.obj = [0.015, 0.015, 0.017]
        # self.obstacle_color = '#416ab6'
        self.setup(args.env)

    def setup(self, env):
        self.grip_w_x = self.safe_areas[env][0]
        self.grip_w_y = self.safe_areas[env][1]
        self.grip_w_z = self.safe_areas[env][2]
        self.obstacle_color = list(reversed(self.obstacle_colors[env]))     # reverse for z-index of the layers

    def createPlot(self, xinit, pred_x, pred_u, k, parameters, obs):
        """Creates a plot and adds the initial data provided by the arguments"""
        x = self.x
        u = self.u
        model = self.model
        sim_length = self.sim_length
        err = self.err

        x[:, k] = xinit  # initial state variables values
        u[:, k] = pred_u[:, 0]

        # Create empty plot
        fig = plt.figure()
        fig.set_size_inches(9, 9, forward=True)
        plt.clf()
        gs = GridSpec(5, 2, figure=fig)

        # Plot trajectory
        ax_pos = fig.add_subplot(gs[:, 0])
        plt.grid("both")
        ax_pos.plot(parameters[0][3], parameters[0][4], 'bo')  # main goal
        plt.title('Position', fontsize=16)
        plt.axis('equal')
        plt.xlim([1.05, 1.4])  # real env set
        plt.ylim([0.4, 1.2])   # real env set
        plt.xlabel('x-coordinate', fontsize=16)
        plt.ylabel('y-coordinate', fontsize=16)
        ax_pos.plot(x[0, 0], x[1, 0], 'b-')     # real trajectory
        ax_pos.plot(pred_x[0, :], pred_x[1, :], 'g-')   # N step forward predicted trajectory
        ax_pos.plot(xinit[0], xinit[1], 'bx')   # initial pose
        ax_pos.plot(parameters[0][0], parameters[0][1], 'kx')  # subgoal

        obst_sz = [model.hl[0] - 0.035, model.hl[1] - 0.035]
        N = parameters.shape[0]

        # Draw obstacles
        real_obstacles = list(obs['real_obstacle_info'])

        for i in range(len(real_obstacles)):
            # real obstacle shape
            obst = real_obstacles[i]
            bbox = (obst[0], obst[1], obst[3] * 2, obst[4] * 2)
            rectangle(bbox, fill=True, linestyle=":", edgecolor='red', color=self.obstacle_color[i])

        # place grip rectangle
        bbox = (xinit[0], xinit[1], (self.obj[0] + self.grip_w_x) * 2, (self.obj[1] + self.grip_w_y) * 2)
        rectangle(bbox, fill=False, linestyle=":", edgecolor='blue')

        for i in range(N - 1):
            # draw convex hull
            p = parameters[i]

            bbox1 = (p[6], p[7], obst_sz[0]*2, obst_sz[0]*2)
            bbox2 = (p[12], p[13], obst_sz[1]*2, obst_sz[1]*2)
            ellipse(bbox1, fill=False, linestyle=":", edgecolor='blue' if i == 0 else 'green',
                    alpha=(1.0 - i / N))
            ellipse(bbox2, fill=False, linestyle=":", edgecolor='blue' if i == 0 else 'green',
                    alpha=(1.0 - i / N))

        # Plot action dx
        ax_dx = fig.add_subplot(gs[0, 1])
        plt.grid("both")
        plt.title('displacement of x', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[0], model.ub[0]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[0], model.lb[0]]), 'r:')
        ax_dx.step(range(0, k + 1), u[0, 0:k + 1], 'b-')
        ax_dx.step(range(k, k + model.N), pred_u[0, :], 'g-')

        # Plot action dy
        ax_dy = fig.add_subplot(gs[1, 1])
        plt.grid("both")
        plt.title('displacement of y', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[1], model.ub[1]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[1], model.lb[1]]), 'r:')
        ax_dy.step(range(0, k + 1), u[1, 0:k + 1], 'b-')
        ax_dy.step(range(k, k + model.N), pred_u[1, :], 'g-')

        # Plot action dz
        ax_dz = fig.add_subplot(gs[2, 1])
        plt.grid("both")
        plt.title('displacement of z', fontsize=16)
        plt.xlim([0., sim_length - 1])
        plt.plot([0, sim_length - 1], np.transpose([model.ub[2], model.ub[2]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([model.lb[2], model.lb[2]]), 'r:')
        ax_dz.step(range(0, k + 1), u[2, 0:k + 1], 'b-')
        ax_dz.step(range(k, k + model.N), pred_u[2, :], 'g-')

        # Plot Z
        ax_Z = fig.add_subplot(gs[3, 1])
        plt.grid("both")
        plt.title('Position Z', fontsize=16)
        plt.xlim([0., sim_length - 1])
        # plt.plot([0, sim_length - 1], np.transpose([model.hu[4], model.hu[4]]), 'r:')
        # plt.plot([0, sim_length - 1], np.transpose([model.hl[4], model.hl[4]]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([0.465, 0.465]), 'r:')
        plt.plot([0, sim_length - 1], np.transpose([0.405, 0.405]), 'r:')
        ax_Z.plot(range(0, k + 1), x[2, 0:k + 1], '-b')
        ax_Z.plot(range(k, k + model.N), pred_x[2, :], 'g-')

        # Plot trajectory
        ax_posZ = fig.add_subplot(gs[4, 1])
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

        # for i in range(len(safe_areas)):
        #     obst = safe_areas[i]
        #     bbox = (obst[1], obst[2], obst[4] * 2, obst[5] * 2)
        #     rectangle(bbox, fill=False, linestyle=":", edgecolor='red')

        plt.tight_layout()
        # Make plot fullscreen. Comment out if platform dependent errors occur.
        mng = plt.get_current_fig_manager()

    def updatePlots(self, next_x, target_x, pred_x, pred_u, k, parameters, ob):
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

        params = parameters[1]
        # Move obstacles
        rectangles = list(filter(lambda obj: type(obj) is matplotlib.patches.Rectangle, ax_list[0].get_children()))
        real_obstacles = list(ob['real_obstacle_info'])

        for i in range(len(real_obstacles)):
            obj = rectangles[i]
            obst = real_obstacles[i]
            obj.set_xy((obst[0] - obst[3], obst[1] - obst[4]))
        obj = rectangles[i + 1]
        obj.set_xy((next_x[0] - (self.obj[0] + self.grip_w_x), next_x[1] - (self.obj[1] + self.grip_w_y)))

        # Move predicted obstacles
        ellipses = list(filter(lambda obj: type(obj) is matplotlib.patches.Ellipse, ax_list[0].get_children()))
        obstacles = []
        for p in parameters[1:]:
            obstacles.append([p[6], p[7]])
            obstacles.append([p[12], p[13]])

        # Draw original and predicted obstacles
        for z in zip(ellipses, obstacles):
            e, o = z
            _, yPos = e.get_center()
            e.center = o[0], o[1]

        # Delete old data in plot
        ax_list[0].get_lines().pop(-1).remove()  # remove old sub goal
        ax_list[0].get_lines().pop(-1).remove()  # remove old init position
        ax_list[0].get_lines().pop(-1).remove()  # remove old prediction of trajectory
        ax_list[0].get_lines().pop(-1).remove()  # remove old trajectory

        ax_list[1].get_lines().pop(-1).remove()  # remove old prediction dx
        ax_list[1].get_lines().pop(-1).remove()  # remove old dx

        ax_list[2].get_lines().pop(-1).remove()  # remove old prediction dy
        ax_list[2].get_lines().pop(-1).remove()  # remove old dy

        ax_list[3].get_lines().pop(-1).remove()  # remove old prediction dz
        ax_list[3].get_lines().pop(-1).remove()  # remove old dz

        ax_list[4].get_lines().pop(-1).remove()  # remove old prediction z
        ax_list[4].get_lines().pop(-1).remove()  # remove old z

        ax_list[5].get_lines().pop(-1).remove()  # remove old prediction of trajectory
        ax_list[5].get_lines().pop(-1).remove()  # remove old trajectory

        # Plot new data in plot
        ax_list[0].plot(x[0, 0:k + 2], x[1, 0:k + 2], '-b')  # plot new trajectory
        ax_list[0].plot(pred_x[0, 1:], pred_x[1, 1:], 'g-')  # plot new prediction of trajectory
        ax_list[0].plot(pred_x[0, 1], pred_x[1, 1], 'mx')  # plot new init position
        ax_list[0].plot(target_x[0], target_x[1], 'kx')  # plot new sub_goal

        ax_list[1].step(range(0, k + 1), u[0, 0:k + 1], 'b-')  # plot new dx
        ax_list[1].step(range(k, k + model.N), pred_u[0, :], 'g-')  # plot new prediction of dx

        ax_list[2].step(range(0, k + 1), u[1, 0:k + 1], 'b-')  # plot new dy
        ax_list[2].step(range(k, k + model.N), pred_u[1, :], 'g-')  # plot new prediction of dy

        ax_list[3].step(range(0, k + 1), u[2, 0:k + 1], 'b-')  # plot new dz
        ax_list[3].step(range(k, k + model.N), pred_u[2, :], 'g-')  # plot new prediction of dz

        ax_list[4].plot(range(0, k + 1), x[2, 0:k + 1], '-b')  # plot new z
        ax_list[4].plot(range(k, k + model.N), pred_x[2, :], 'g-')  # plot new prediction of z

        ax_list[5].plot(params[1], params[2], 'kx')  # plot new subgoal position
        ax_list[5].plot(x[1, 0:k + 2], x[2, 0:k + 2], '-b')  # plot new trajectory
        ax_list[5].plot(pred_x[1, 1:], pred_x[2, 1:], 'g-')  # plot new prediction of trajectory

    def show(self):
        plt.show()

    def draw(self):
        plt.draw()





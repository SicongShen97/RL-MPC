from franka_robot import FrankaRobot
import numpy as np
import time
# from policies_real.franka_mpc_policy import MPCPolicy

# policy = MPCPolicy()
# policy.initialize()
# policy.play()

def init(robot, pose):
    robot.move_to_init(pose)
    input("Enter to lower the gripper.")
    disp_z = pose[2] - 0.02
    robot.move([0, 0, -disp_z])
    input("Enter to grasp the object.")
    robot.clamp()
    robot.move([0, 0, disp_z])
    input("Enter to release the object.")
    robot.release()



robot = FrankaRobot()
cur_pose = robot.current_pose()
print(cur_pose)
# disp_z = cur_pose - 0.02
# robot.move([0, 0, -0.01])
# pose = [0.45, 0.35, 0.07]
robot.move_to_init([0.5, 0.3, 0.1])
# robot.release()
# init = np.array([0.5, 0.3])
# robot.move([0, 0, 0.2])
# z = robot.current_pose()[2]
# pose = np.append(init, z)
# robot.move_to_init(pose)
# input("Enter to lower the gripper.")
# pose = np.append(init, 0.01)
# robot.move_to_init(pose)
input("Enter to grasp the object.")
# robot.clamp()
robot.move([0, 0, 0.2])
# input("Enter to start moving.")
# pose = robot.current_pose()
# print(pose)

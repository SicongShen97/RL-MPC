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
disp_z = cur_pose - 0.02
robot.move([0, 0, -0.01])
pose = [0.45, 0.35, 0.07]

robot.release()

# pose = robot.current_pose()
# print(pose.vector()[0])

from frankx import Affine, LinearRelativeMotion, Robot, JointMotion, LinearMotion

class FrankaRobot:
    def __init__(self, id = "192.168.5.12"):
        self.robot = Robot(id)
        self.gripper = self.robot.get_gripper()
        self.robot.set_default_behavior()
        self.robot.velocity_rel = 1
        self.robot.acceleration_rel = 0.5
        self.robot.jerk_rel = 0.01
        self.robot.recover_from_errors()

    def move_to_init(self, initial_pos):
        self.robot.move(LinearMotion(Affine(*initial_pos, 0, 0, 0)))
        # self.robot.move(LinearMotion(Affine(0, 0, -0.03, 0, 0, 0)))

    def clamp(self):
        self.gripper.clamp()

    def release(self, disp=0.05):
        self.gripper.release(disp)

    def move(self, displacement):
        self.robot.move(LinearRelativeMotion(Affine(*displacement)))

    def current_pose(self):
        return self.robot.current_pose().vector()


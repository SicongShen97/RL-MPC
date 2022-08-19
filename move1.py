#!/usr/bin/python3 -u
from concurrent.futures import thread
from utils_mios.ws_client import *
from mongodb_client import MongoDBClient
import time
import json
import os
import nest_asyncio
import numpy as np
nest_asyncio.apply()

I_3x3_List = [1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0]

class Task:
    def __init__(self, robot):
        self.skill_names = []
        self.skill_types = []
        self.skill_context = dict()

        self.robot = robot
        self.task_uuid = "INVALID"
        self.t_0 = 0

    def add_skill(self, name, type, context):
        self.skill_names.append(name)
        self.skill_types.append(type)
        self.skill_context[name] = context

    def start(self, queue: bool = False):
        self.t_0 = time.time()
        parameters = {
            "parameters": {
                "skill_names": self.skill_names,
                "skill_types": self.skill_types,
                "as_queue": queue
            },
            "skills": self.skill_context
        }
        print("start skill : ", self.skill_names)
        response = start_task(self.robot, "GenericTask", parameters)
        self.task_uuid = response["result"]["task_uuid"]

    def wait(self):
        result = wait_for_task(self.robot, self.task_uuid)
        return result

def move_cart(robot, pos_offset, pose="EndEffector"):
    move_up_context = {
        "skill": {
            "objects": {
                "goal_pose": pose  
            },
            "speed": [0.1, 0.1],
            "acc": [0.2, 0.2],
            "T_T_EE_g_offset": [1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                pos_offset[0], pos_offset[1], pos_offset[2], 1],
        },
        "control": {
            "control_mode": 2,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            },
        },
        "user": {"F_ext_max": [15, 15]}
    }
    t = Task(robot)
    t.add_skill("move_cart", "MoveToPoseCart", move_up_context)
    t.start()
    return t.wait()
    
def push_z(robot):
    path_to_default_context = os.getcwd() + "/default_contexts/"
    f = open(path_to_default_context + "push_surface.json")
    default_context = json.load(f)
    default_context["skill"]["objects"]["Surface"] = "press_userstop_button"
    default_context["skill"]["objects"]["Approach"] = "press_userstop_approach"
    print(default_context)
    t = Task(robot)
    t.add_skill("skill_name", "TaxPush", default_context)
    t.start(queue=False)
    return t.wait()

def move_joint(robot, pose="NullObject"):
    joint_move_context = {
        "skill": {
            "objects": {
                "goal_pose": pose
            },
            "speed": 0.1,
            "acc": 0.5,
        },
        "control": {
            "control_mode": 1,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 250, 250, 250]
            }
        },
        "user": {"F_ext_max": [15, 15]}
    }
    t = Task(robot)
    t.add_skill("move_to_joint_pose", "MoveToPoseJoint", joint_move_context)
    t.start()
    return t.wait()


def move_cart_delta_R(robot, rotation_list):
    """ Move to a specific height(target_z) and pose(rotation_matrix=I) for following visual perception"""
    # mongodb = mongodb_client.MongoDBClient(robot)
    res = call_method(robot, 12000, "get_state")
    # set height
    ee_array = np.array(res["result"]["O_T_EE"])
    ee_matrix = (ee_array.reshape((4, 4))).T
    print(ee_matrix)
    # ee_array[14] = target_z

    # set rotation
    ee_array[0:12] = np.array(rotation_list)
    print((ee_array.reshape((4, 4))).T)

    call_method(robot, 12000, "set_object", {"object": "ssc", "O_T_OB": list(ee_array)})
    move_cart(robot, pos_offset=[0.0, 0.0, 0.0], pose="ssc")


def move_cart_delta_T(robot, pos_offset, translation_list):
    # mongodb = mongodb_client.MongoDBClient(robot)
    res = call_method(robot, 12000, "get_state")
    ee_array = np.array(res["result"]["O_T_EE"])
    ee_matrix = (ee_array.reshape((4, 4))).T
    ee_array[12] += translation_list[0]
    ee_array[13] += translation_list[1]
    ee_array[14] += translation_list[2]
    # print(ee_matrix)
    # mongodb.update("mios", "environment", {"name": "position_2"}, {"O_T_OB": list(ee_matrix)})
    call_method(robot, 12000, "set_object", {"object": "position_2", "O_T_OB": list(ee_array)})
    move_cart(robot, pos_offset, pose="position_2")

def move_to_contact(robot):
    move_to_contact_context = {
            "skill": {
                "objects": {
                    "goal_pose": "EndEffector"#"hole_40"#"table"
                        },
                "speed": 0.05
                    },
            "control": {
                "control_mode": 0
                    },
            "user":{
                "F_ext_contact": [4.0, 2.0],
                #"F_ext_max": [10, 5]
                #"env_X": [0.001,0.001]
            }
                }
    t = Task(robot)
    t.add_skill("look_for_contact", "MoveToContact", move_to_contact_context)
    t.start()
    t.wait()

def teach_position(robot, position_name, teach_gripper_width=False):
    return call_method(robot, 12000, "teach_object", {"object": position_name, "teach_width": teach_gripper_width})

def grasp(robot):
    # grasp sth smaller than 10cm (epsilon_outer=0.1)
    return call_method(robot, 12000, "grasp", {"width": 0.0, "speed": 1, "force": 100, "epsilon_inner": 1, "epsilon_outer": 0.1})

def open_gripper(robot):
    # opens the gripper completly    
    return call_method(robot, 12000, "release_object", {"speed":1})

def example(robot):
    # response = call_method(robot, 12000, "get_state")
    # print(response)
    # response = push_z(robot)
    response = move_cart(robot, "position_default")
    # move_joint(robot, "position_default")
    # print(response)
    # teach
    # input("Move to position default \n press Enter if done.")
    # teach_position(robot, "position_default")
    # input("Move to pickup pose \n press Enter if done.")
    # teach_position(robot, "position_1")
    # input("Move to place pose \n press Enter if done.")
    # teach_position(robot, "position_2")
    # input("Good work. Robot will now execute the task (Enter)")

    # move
    # open_gripper(robot)
    # move_joint(robot,"position_default")
    # move_cart(robot, [0,0,0], "position_1")
    # response = start_task(robot, "MoveToCartPose", parameters={"parameters": {"speed": [0.1, 0.3], "acc": [0.1, 0.3], "pose":"position_1"}})
    # grasp(robot)
    # move_cart(robot,[0,0,0],"position_default")
    # move_cart(robot,[0,0,0],"position_2")
    # open_gripper(robot)
    # move_joint(robot,"position_default")

def print_pos(robot):
    res = call_method(robot, 12000, "get_state")
    ee_array = np.array(res["result"]["O_T_EE"])
    # print("array:", ee_array)
    print(res["result"])
    print("*"*20)
    print(ee_array.reshape((4, 4)).T)

def teach_move_to_begin(robot, pos, offset=[0.8, 0.75, 0.38]):
    res = call_method(robot, 12000, "get_state")
    ee_array = np.array(res["result"]["O_T_EE"])
    rotation_list = [1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0]
    ee_array[:12] = np.array(rotation_list)
    ee_array[12] = pos[0] - offset[0]
    ee_array[13] = pos[1] - offset[1]
    ee_array[14] = pos[2] - offset[2]
    call_method(robot, 12000, "set_object", {"object": "ssc", "O_T_OB": list(ee_array)})
    move_cart(robot, pos_offset=[0, 0, 0], pose="ssc")


if __name__ == '__main__':
    robot = "192.168.5.1"
    # I_3x3_List = [1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0]
    # teach_move_to_begin(robot)
    # n = 10
    # delta_all = [0, -0.1*n, 0]
    # offset = [0, 0, 0]
    # # for _ in range(5):
    # t1 = time.time()
    # move_cart_delta_T(robot, pos_offset=offset, translation_list=delta_all)
    # t2 = time.time()
    # t_no_stop = t2-t1
    #
    # delta = [0, -0.1, 0]
    # teach_move_to_begin(robot)
    # t1 = time.time()
    # for _ in range(n):
    #     move_cart_delta_T(robot, pos_offset=offset, translation_list=delta)
    # t2 = time.time()
    # t_stop = t2-t1
    # print((t_stop - t_no_stop)/(n-1))

    # move_cart_delta_R(robot, rotation_list=I_3x3_List)
    # move_cart_delta_T(robot, translation_list=[0.1, 0.1, 0])
    # print_pos(robot)
    # example(robot)
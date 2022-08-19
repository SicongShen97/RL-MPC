#!/usr/bin/python3 -u
import time
import numpy as np

from utils.ws_client import *
from xmlrpc.client import ServerProxy

import csv


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
        response = start_task(self.robot, "GenericTask", parameters={
            "parameters": {
                "skill_names": self.skill_names,
                "skill_types": self.skill_types,
                "as_queue": queue
            },
            "skills": self.skill_context
        })
        self.task_uuid = response["result"]["task_uuid"]

    def start_multiple(self, queue: bool = False):
        self.t_0 = time.time()
        if isinstance(self.robot, list):
            responses = start_multiple_task(self.robot, "GenericTask", parameters={
                "parameters": {
                    "skill_names": self.skill_names,
                    "skill_types": self.skill_types,
                    "as_queue": queue
                },
                "skills": self.skill_context
            })
            # TODO: how to avoid response to become a set?
            responses_list = list(responses[0])
            r1 = responses_list[0]
            self.task_uuid = [responses_list[i]._result["result"]["task_uuid"] for i in range(len(self.robot))]
            # for r in response:
            #     self.task_uuid.append(r["result"]["task_uuid"])
        else:
            raise Exception('Robot address is not a list. The value of robot was: {}'.format(self.robot))

    def wait(self):
        result = wait_for_task(self.robot, self.task_uuid)
        print("Task execution took " + str(time.time() - self.t_0) + " s.")
        print(self.task_uuid)
        return result

    def wait_multiple(self):
        if isinstance(self.robot, list):
            for i in range(len(self.robot)):
                result = wait_for_task(self.robot[i], self.task_uuid[i])
                print("Task execution took " + str(time.time() - self.t_0) + " s.")
                print(self.task_uuid[i])
        else:
            raise Exception('Robot address is not a list.')
        return result


def start_skill(address: str, skill: str, parameters: dict, control: dict, user: dict = {}, skill_name: str = "skill"):
    response = start_task(address, "GenericTask", parameters={"parameters": {
        "skill_names": [skill_name],
        "skill_types": [skill]
    },
        "skills": {
            "skill": {
                "skill": parameters,
                "control": control,
                "user": user
            }
        }})
    return response


def tax_test_grab(robot="collective-panda-008.local"):
    grab_context = {
        "skill": {
            "objects": {
                "Retract": "iros_key_grab_retract",
                "Approach": "iros_key_grab_approach",
                "Grabbable": "iros_key"
            },
            "time_max": 5,
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4],
            "grab_speed": [0.05, 0.1],
            "grab_acc": [0.1, 0.4],
            "grasp_width": 0.03,
            "grasp_speed": 0.2,
            "grasp_force": 30,
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [1900, 1900, 1900, 190, 190, 190]
            }
        }
    }
    t = Task(robot)
    t.add_skill("grab", "TaxGrab", grab_context)
    t.start()
    result = t.wait()
    cost = result["result"]["task_result"]["skill_results"]["grab"]["cost"]["time"]
    heuristic = result["result"]["task_result"]["skill_results"]["grab"]["heuristic"]
    total_cost = cost + heuristic
    with open("expert_data_grab.txt", "a") as f:
        f.write(str(total_cost) + "\n")

    print("Total cost: " + str(total_cost))


def tax_test_place(robot="collective-panda-008.local"):
    call_method(robot, 12000, "set_grasped_object", {"object": "iros_key"})
    place_context = {
        "skill": {
            "objects": {
                "Retract": "iros_key_grab_retract",
                "Approach": "iros_key_grab_approach",
                "Placeable": "iros_key",
                "Surface": "iros_key_storage"
            },
            "time_max": 5,
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4],
            "place_speed": [0.5, 1.0],
            "place_acc": [1., 4.0],
            "release_width": 0.06,
            "release_speed": 2.0,
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.015, 0.02]
        }
    }
    t = Task(robot)
    t.add_skill("place", "TaxPlace", place_context)
    t.start()
    result = t.wait()
    cost = result["result"]["task_result"]["skill_results"]["place"]["cost"]["time"]
    heuristic = result["result"]["task_result"]["skill_results"]["place"]["heuristic"]
    total_cost = cost + heuristic
    with open("expert_data_place.txt", "a") as f:
        f.write(str(total_cost) + "\n")

    print("Total cost: " + str(total_cost))


def tax_test_turn(robot="collective-panda-008.local"):
    call_method(robot, 12000, "set_grasped_object", {"object": "iros_key"})
    turn_context = {
        "skill": {
            "objects": {
                "Turnable": "iros_key",
                "GoalOrientation": "iros_turn_goal"
            },
            "turn_speed": [0.5, 2.5],
            "turn_acc": [2, 25]},
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.01, 0.02]
        }
    }
    turn_back_context = {
        "skill": {
            "objects": {
                "Turnable": "iros_key",
                "GoalOrientation": "iros_lock"
            },
            "turn_speed": [0.2, 2.5],
            "turn_acc": [0.5, 30.0]},
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.01, 0.02]
        }
    }
    t = Task(robot)
    t.add_skill("turn", "TaxTurn", turn_context)
    #t.add_skill("turn_back", "TaxTurn", turn_back_context)
    t.start()
    result = t.wait()
    cost = result["result"]["task_result"]["skill_results"]["turn"]["cost"]["time"]
    heuristic = result["result"]["task_result"]["skill_results"]["turn"]["heuristic"]
    total_cost = cost + heuristic
    with open("expert_turn_place.txt", "a") as f:
        f.write(str(total_cost) + "\n")

    print("Total cost: " + str(total_cost))


def test_tax_press_button(robot="collective-panda-008.local"):
    start_skill(robot, "TaxPressButton",
                {"objects": {"Button": "iros_button", "Approach": "iros_button_approach"},
                 "approach_speed": [0.05, 0.5], "approach_acc": [0.5, 1.0],
                 "press_speed": [0.05, 0.5], "press_acc": [0.5, 1.0], "duration": 2,
                 "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
                 "ROI_phi": [0, 0, 0, 0, 0, 0],
                 "condition_level_success": "External",
                 "condition_level_error": "External"
                 },
                {"control_mode": 0},
                {
                    "env_X": [0.005, 0.1]
                })


def tax_test_move(robot, location):
    move1_context = {
        "skill": {
            "objects": {
                "GoalPose": location
            },
            "speed": [0.1, 0.5],
            "acc": [0.5, 1.0]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        }
    }
    t = Task(robot)
    t.add_skill("move", "TaxMove", move1_context)
    t.start()
    result = t.wait()


def tax_test_insertion(robot):
    call_method(robot, 12000, "set_grasped_object", {"object": "generic_insertable"})
    insertion_context = {
        "skill": {
            "objects": {
                "Container": "generic_container",
                "Approach": "generic_container_approach",
                "Insertable": "generic_insertable"
            },
            "objects_modifier": {
                "Approach": {
                    "T_T_OB": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0.05, 0, 1]
                }
            },
            "time_max": 10,
            "approach_speed": [0.1, 1],
            "approach_acc": [0.5, 4],
            "insertion_speed": [0.1, 0.5],
            "insertion_acc": [0.5, 1],
            "f_max_push": 0,
            "DeltaX": [0, 0, 0, 0, 0, 0],
            "search_a": [0, 0, 0, 0, 0, 0],
            "search_f": [0, 0, 0, 0, 0, 0],
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0],
            "stuck_dx_thr": 0.05
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        }
    }
    t = Task(robot)
    t.add_skill("insertion", "TaxInsertion", insertion_context)
    t.start()
    result = t.wait()
    cost = result["result"]["task_result"]["skill_results"]["insertion"]["cost"]["time"]
    heuristic = result["result"]["task_result"]["skill_results"]["insertion"]["heuristic"]
    total_cost = cost + heuristic
    with open("expert_data_insertion.txt", "a") as f:
        f.write(str(total_cost) + "\n")

    print("Total cost: " + str(total_cost))


def tax_test_button_press(robot):
    button_press_context = {
        "skill": {
            "objects": {
                "Button": "iros_button",
                "Approach": "iros_button_approach"
            },
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4],
            "press_speed": [0.5, 1],
            "f_push": 1,
            "press_acc": [1, 4],
            "duration": 0,
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0],
            "condition_level_success": "External",
            "condition_level_error": "External"
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.01, 0.02]
        }
    }
    # s = ServerProxy("http://localhost:8000", allow_none=True)
    # s.subscribe_to_event("button_press", "collective-panda-010.local", "12000")
    t = Task(robot)
    t.add_skill("button_press", "TaxPressButton", button_press_context)
    t.start()
    result = t.wait()
    cost = result["result"]["task_result"]["skill_results"]["button_press"]["cost"]["time"]
    heuristic = result["result"]["task_result"]["skill_results"]["button_press"]["heuristic"]
    total_cost = cost + heuristic
    with open("expert_data_button_press.txt", "a") as f:
        f.write(str(total_cost) + "\n")

    print("Total cost: " + str(total_cost))


def subscribe_to_event_server(robot):
    s = ServerProxy("http://collective-control-001.local:8002", allow_none=True)
    s.subscribe_to_event("button_press", robot, "12000")


def tax_test_extraction(robot="collective-panda-008.local"):
    call_method(robot, 12000, "set_grasped_object", {"object": "generic_insertable"})
    extraction_context = {
        "skill": {
            "objects": {
                "Container": "generic_container",
                "ExtractTo": "generic_container_approach",
                "Extractable": "generic_insertable"
            },
            "time_max": 5,
            "extraction_speed": [0.05, 0.15],
            "extraction_acc": [1, 0.8],
            "search_a": [5, 5, 0, 2, 2, 0],
            "search_f": [0.5, 0.5, 0, 2, 2, 0],
            "stuck_dx_thr": 0.05
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [500, 500, 2000, 20, 20, 20]
            }
        }
    }
    t = Task(robot)
    t.add_skill("extract", "TaxExtraction", extraction_context)
    t.start()
    result = t.wait()
    cost = result["result"]["task_result"]["skill_results"]["extract"]["cost"]["time"]
    heuristic = result["result"]["task_result"]["skill_results"]["extract"]["heuristic"]
    total_cost = cost + heuristic
    with open("expert_data_extraction.txt", "a") as f:
        f.write(str(total_cost) + "\n")

    print("Total cost: " + str(total_cost))


def test_skill_queue(robot="localhost"):
    move1_context = {
        "skill": {
            "objects": {
                "GoalPose": "test_pose_1"
            },
            "speed": [0.1, 0.5],
            "acc": [0.5, 5]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "frames": {
            "O_R_T": [1, 0, 0, 0, 1, 0, 0, 0, 1]
        }
    }
    move2_context = {
        "skill": {
            "objects": {
                "GoalPose": "test_pose_2"
            },
            "speed": [0.1, 0.5],
            "acc": [0.5, 5]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [1000, 1000, 1000, 100, 100, 100]
            }
        },
        "frames": {
            "O_R_T": [1, 0, 0, 0, -1, 0, 0, 0, -1]
        }
    }
    t = Task(robot)
    t.add_skill("move1", "TaxMove", move1_context)
    t.add_skill("move2", "TaxMove", move2_context)
    t.add_skill("move3", "TaxMove", move1_context)
    t.add_skill("move4", "TaxMove", move2_context)
    t.add_skill("move5", "TaxMove", move1_context)
    t.add_skill("move6", "TaxMove", move2_context)
    t.start(True)
    result = t.wait()
    print(result)


def iros_task():
    robot = "collective-panda-010.local"

    response = start_task(robot, "MoveToJointPose", {"parameters": {"pose": "iros_idle_pose"}})
    wait_for_task(robot, response["result"]["task_uuid"])
    t_0 = time.time()
    # move to grab key
    iros1 = Task(robot)

    grab_context = {
        "skill": {
            "ignore_settling": True,
            "objects": {
                "Retract": "iros_key_grab_retract",
                "Approach": "iros_key_grab_approach",
                "Grabbable": "iros_key"
            },
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4.0],
            "grab_speed": [0.3, 2],
            "grab_acc": [1, 4],
            "grasp_width": 0.0,
            "grasp_speed": 100,
            "grasp_force": 30,
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        }
    }

    insertion_context = {
        "skill": {
            "objects": {
                "Container": "iros_lock",
                "Approach": "iros_lock_approach",
                "Insertable": "iros_key"
            },
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4],
            "insertion_speed": [0.2, 0.5],
            "insertion_acc": [0.8, 1.0],
            "search_a": [3, 3, 0, 0, 0, 0],
            "search_f": [1, 0.75, 0, 0, 0, 0],
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0],
            "f_max_push": 5
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [500, 500, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.02, 0.04]
        }
    }
    turn_context = {
        "skill": {
            "ignore_settling": True,
            "objects": {
                "Turnable": "iros_key",
                "GoalOrientation": "iros_turn_goal"
            },
            "turn_speed": [0.2, 2],
            "turn_acc": [0.5, 30.0]},
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.03, 0.02]
        }
    }
    turn_back_context = {
        "skill": {
            "objects": {
                "Turnable": "iros_key",
                "GoalOrientation": "iros_lock"
            },
            "turn_speed": [0.2, 2],
            "turn_acc": [0.5, 30.0]},
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },"user": {
            "env_X": [0.03, 0.02]
        }
    }
    extraction_context = {
        "skill": {
            "ignore_settling": True,
            "objects": {
                "Container": "iros_lock",
                "ExtractTo": "iros_lock_retract",
                "Extractable": "iros_key"
            },
            "extraction_speed": [0.5, 4],
            "extraction_acc": [1, 1.0],
            "search_a": [0, 0, 0, 0, 0, 0],
            "search_f": [0, 0, 0, 0, 0, 0]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200,200, 200]
            }
        },
        "user": {
            "env_X": [0.01, 0.02]
        }
    }

    place_context = {
        "skill": {
            "ignore_settling": True,
            "objects": {
                "Retract": "iros_key_place_retract",
                "Approach": "iros_key_place_approach",
                "Placeable": "iros_key",
                "Surface": "iros_key_storage"
            },
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4],
            "place_speed": [0.1, 0.5],
            "place_acc": [0.5, 1.0],
            "release_width": 0.05,
            "release_speed": 100,
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0]
        },
        "control": {
            "control_mode": 0
        },
        "user": {
            "env_X": [0.015, 0.02]
        }
    }
    move4_context = {
        "skill": {
            "ignore_settling": True,
            "objects": {
                "GoalPose": "iros_button_roi"
            },
            "speed": [0.5, 1],
            "acc": [2, 4],
            "finger_width": 0,
            "finger_speed": 100
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        },
        "user": {
            "env_X": [0.02, 0.04]
        }
    }
    button_press_context = {
        "skill": {
            "ignore_settling": True,
            "objects": {
                "Button": "iros_button",
                "Approach": "iros_button_approach"
            },
            "f_push": 5,
            "approach_speed": [0.5, 1],
            "approach_acc": [1, 4],
            "press_speed": [0.1, 0.5],
            "press_acc": [1, 1.0],
            "duration": 0,
            "ROI_x": [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "ROI_phi": [0, 0, 0, 0, 0, 0]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
         },
        "user": {
            "env_X": [0.01, 0.04]
        }
    }
    move5_context = {
        "skill": {
            "objects": {
                "GoalPose": "iros_idle_pose"
            },
            "speed": [0.3, 0.5],
            "acc": [1, 1]
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        }
    }

    iros1.add_skill("grab_key", "TaxGrab", grab_context)
    iros1.add_skill("insertion", "TaxInsertion", insertion_context)
    iros1.add_skill("turn_key", "TaxTurn", turn_context)
    iros1.add_skill("turn_back_key", "TaxTurn", turn_back_context)
    iros1.add_skill("extraction", "TaxExtraction", extraction_context)
    iros1.add_skill("place_key", "TaxPlace", place_context)
    iros1.add_skill("move_to_button", "TaxMove", move4_context)
    iros1.add_skill("press_button", "TaxPressButton", button_press_context)
    iros1.add_skill("move_to_idle", "TaxMove", move5_context)

    iros1.start(True)
    result = iros1.wait()

    print(result)
    print("Execution time: " + str(time.time() - t_0))

    cost = dict()

    for skill, r in result["result"]["task_result"]["skill_results"].items():
        cost[skill] = r["cost"]["time"]

    return cost


def iros_task_loop():
    cost_avg = dict()
    for i in range(10):
        cost = iros_task()
        for skill, c in cost.items():
            if skill not in cost_avg:
                cost_avg[skill] = []
            cost_avg[skill].append(c)

    with open('iros_data.csv', 'w') as f:
        write = csv.writer(f)
        write.writerow(cost_avg.keys())
        table = []
        i = 0
        for skill, c in cost_avg.items():
            table.append(c)
            i += 1

        table_np = np.asarray(table)
        table = table_np.transpose().tolist()
        write.writerows(table)


def test_draw(robot="collective-panda-008.local"):
    call_method(robot, 12000, "set_grasped_object", {"object": "test_draw_pen"})
    draw_context = {
        "skill": {
            "objects": {
                "Pen": "test_draw_pen",
                "Surface": "test_draw_surface"
            },
            "approach_speed": [0.2, 0.5],
            "approach_acc": [0.5, 1],
            "contact_speed": [0.05, 0.5],
            "contact_acc": [0.5, 1],
            "draw_speed": [0.2, 0.5],
            "draw_acc": [0.5, 1],
            "file_mode": True,
            "path_file": "painting.txt",
            "f_draw": 10,
            "surface_distance": 0.05,
            "port_src": 8888
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [1500, 1500, 1500, 150, 150, 150]
            },
            "nullspace_control": {
                "K_theta": [20, 20, 15, 10, 7, 5, 2],
                "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                "active": True
            }
        },
        "user": {
            "F_ext_contact": [8, 5]
        }
    }
    t = Task(robot)
    t.add_skill("draw", "Draw", draw_context)
    t.start()
    result = t.wait()


def test_crank(robot="collective-panda-008.local"):
    call_method(robot, 12000, "set_grasped_object", {"object": "paternoster_crank_handle"})
    crank_context = {
        "skill": {
            "objects": {
                "Crank": "paternoster_crank_handle",
                "Center": "paternoster_crank_center"
            },
            "crank_speed": [0.01, 0.5],
            "crank_acc": [0.5, 1],
            "radius": 0.1,
            "n_turns": 2,
            "clockwise": False
        },
        "control": {
            "control_mode": 0,
            "cart_imp": {
                "K_x": [1500, 1500, 1500, 150, 150, 150]
            },
            "nullspace_control": {
                "K_theta": [20, 20, 15, 10, 7, 5, 2],
                "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                "active": True
            }
        }
    }
    t = Task(robot)
    t.add_skill("crank", "Crank", crank_context)
    t.start()
    result = t.wait()


def move_to_pose(robot, pose):
    move_context = {
        "skill": {
            "objects": {
                "goal_pose": pose
            },
            "speed": 0.5,
            "acc": 1.0
        },
        "control": {
            "control_mode": 3,
        }
    }
    t = Task(robot)
    t.add_skill("move", "MoveToPoseJoint", move_context)
    t.start()
    result = t.wait()


def move_to_pose_2(robot, pose):
    move_context = {
        "skill": {
            "objects": {
                "GoalPose": pose
            },
            "speed": [0.1, 0.5],
            "acc": [0.5, 1.0]
        },
        "control": {
            "control_mode": 2,
            "cart_imp": {
                "K_x": [2000, 2000, 2000, 200, 200, 200]
            }
        }
    }
    t = Task(robot)
    t.add_skill("move", "TaxMove", move_context)
    t.start()
    result = t.wait()

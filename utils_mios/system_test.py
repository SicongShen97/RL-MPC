#!/usr/bin/python3 -u
import time
import numpy as np
import json
import socket

from ws_client import *


# from udp_client import *
# from rpc_client import *


class TestException(Exception):
    pass


def msg_error(assertion, test, msg, rtn):
    if assertion is False:
        print('Error during test: ' + test)
        print(rtn)
        raise TestException(msg)


def system_diagnose(address):
    try:
        test_exception_handling(address)
        test_start_stop(address)
        test_start_wait(address)
        test_primitive_sequence(address)
        test_multilayer_tasks(address)
        test_subtask_start_stop(address)
        #test_task_queue(address)
        test_skill_parallel(address)
        test_live_parameters(address)
        test_memory(address)
        print("System diagnose has revealed no errors.")
    except TestException as e:
        print(e)
        print("System diagnose has failed.")


def test_exception_handling(hostname):
    address = hostname
    print('Testing exception handling...')

    rtn = start_task(address, 'TestTask1', {"parameters": {"exception": "task", "success": True}})
    msg_error(rtn is not None, 'exception_handling_task', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'exception_handling_task', 'Result is false', rtn)
    msg_error('task_uuid' in rtn['result'], 'exception_handling_task', 'Result has no unique task id', rtn)

    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["result_code"] == 9, "exception_handling_task",
              "Wrong result code", rtn)

    rtn = start_task(address, 'TestTask1', {"parameters": {'exception': "skill", "success": True}})
    msg_error(rtn is not None, 'exception_handling_skill', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'exception_handling_skill', 'Result is false', rtn)
    msg_error('task_uuid' in rtn['result'], 'exception_handling_skill', 'Result has no unique task id', rtn)

    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["t1_s1"]["result_code"] == 5, "exception_handling_skill",
              "Wrong result code", rtn)

    rtn = start_task(address, 'TestTask1', {"parameters": {'exception': "control", "success": True}})
    msg_error(rtn is not None, 'exception_handling_control', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'exception_handling_control', 'Result is false', rtn)
    msg_error('task_uuid' in rtn['result'], 'exception_handling_control', 'Result has no unique task id', rtn)

    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["t1_s1"]["result_code"] == 1, "exception_handling_control",
              "Wrong result code", rtn)

    rtn = start_task(address, 'TestTask1', {"parameters": {'exception': "invalid", "success": True}})
    msg_error(rtn is not None, 'exception_handling_invalid', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'exception_handling_invalid', 'Result is false', rtn)
    msg_error('task_uuid' in rtn['result'], 'exception_handling_invalid', 'Result has no unique task id', rtn)

    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["t1_s1"]["result_code"] == 2, "exception_handling_invalid",
              "Wrong result code", rtn)

    rtn = start_task(address, 'TestTask1', {"parameters": {'exception': "network", "success": True}})
    msg_error(rtn is not None, 'exception_handling_network', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'exception_handling_network', 'Result is false', rtn)
    msg_error('task_uuid' in rtn['result'], 'exception_handling_network', 'Result has no unique task id', rtn)

    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["t1_s1"]["result_code"] == 3, "exception_handling_network",
              "Wrong result code", rtn)

    rtn = start_task(address, 'TestTask1', {"parameters": {"exception": "realtime", "success": True}})
    msg_error(rtn is not None, 'exception_handling_realtime', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'exception_handling_realtime', 'Result is false', rtn)
    msg_error('task_uuid' in rtn['result'], 'exception_handling_realtime', 'Result has no unique task id', rtn)

    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["t1_s1"]["result_code"] == 4, "exception_handling_realtime",
              "Wrong result code", rtn)


def test_start_stop(hostname):
    address = hostname
    url = "http://" + hostname + ":8383"

    # start and various stops after 1 second
    print('Testing exceptional stop...')
    rtn = start_task(address, "TestTask1", queue=True)
    msg_error(rtn['result']['result'], 'start_stop_exception', 'Result is false', rtn)
    task_uuid = rtn["result"]["task_uuid"]
    time.sleep(1)
    rtn = stop_task(address, True)
    msg_error(rtn is not None, 'start_stop_exception', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'start_stop_exception', 'Result is false', rtn)
    rtn = wait_for_task(address, task_uuid)

    msg_error(rtn['result']['task_result']["exception"] is True, 'start_stop_exception', 'Result has no exception', rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'start_stop_exception', 'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "start_stop_exception",
              "Has recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "start_stop_exception",
              "No external stop", rtn)

    print('Testing exceptional stop with recovery...')
    rtn = start_task(address, "TestTask1", queue=True)
    msg_error(rtn['result']['result'], 'start_stop_exception', 'Result is false', rtn)
    task_uuid = rtn["result"]["task_uuid"]
    time.sleep(1)
    rtn = stop_task(address, True, recover=True)
    msg_error(rtn is not None, 'start_stop_exception', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'start_stop_exception', 'Result is false', rtn)
    rtn = wait_for_task(address, task_uuid)

    msg_error(rtn['result']['task_result']["exception"] is True, 'start_stop_exception_recovery',
              'Result has no exception', rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'start_stop_exception_recovery', 'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "start_stop_exception_recovery",
              "Has recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "start_stop_exception_recovery",
              "No external stop", rtn)

    print('Testing stop without recovery...')
    rtn = start_task(address, "TestTask1", queue=True)
    msg_error(rtn['result']['result'], 'start_stop_unsuccessful', 'Result is false', rtn)
    task_uuid = rtn["result"]["task_uuid"]
    time.sleep(1)
    rtn = stop_task(address)
    msg_error(rtn is not None, 'start_stop_unsuccessful', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'start_stop_unsuccessful', 'Result is false', rtn)
    rtn = wait_for_task(address, task_uuid)

    msg_error(rtn['result']['task_result']["exception"] is False, 'start_stop_unsuccessful', 'Result has exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'start_stop_unsuccessful', 'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "start_stop_unsuccessful",
              "Recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "start_stop_unsuccessful",
              "No external stop", rtn)

    print('Testing stop with recovery...')
    rtn = start_task(address, "TestTask1", queue=True)
    msg_error(rtn['result']['result'], 'start_stop_unsuccessful_recovery', 'Result is false', rtn)
    task_uuid = rtn["result"]["task_uuid"]
    time.sleep(1)
    rtn = stop_task(address, recover=True)
    msg_error(rtn is not None, 'start_stop_unsuccessful_recovery', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'start_stop_unsuccessful_recovery', 'Result is false', rtn)
    rtn = wait_for_task(address, task_uuid)
    msg_error(rtn['result']['task_result']["exception"] is False, 'start_stop_unsuccessful_recovery',
              'Result has exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'start_stop_unsuccessful_recovery',
              'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is True, "start_stop_unsuccessful_recovery",
              "Not recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "start_stop_unsuccessful_recovery",
              "No external stop", rtn)


def test_start_wait(hostname):
    address = hostname
    url = "http://" + hostname + ":8383"

    print('Testing execution with successful stop...')
    rtn = start_task(address, "TestTask1", {"parameters": {"success": True}}, queue=True)
    msg_error(rtn['result']['result'], 'start_wait_success', 'Result is false', rtn)
    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn['result']['task_result']["exception"] is False, 'start_stop_success_recovery', 'Result has exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is True, 'start_stop_success_recovery',
              'Result is not successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "start_stop_success_recovery",
              "Not recovered", rtn)

    print('Testing execution with unsuccessful stop...')
    rtn = start_task(address, "TestTask1", {"parameters": {"success": False}}, queue=True)
    msg_error(rtn['result']['result'], 'start_wait_success', 'Result is false', rtn)
    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn['result']['task_result']["exception"] is False, 'start_stop_success_recovery', 'Result has exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'start_stop_success_recovery',
              'Result is not successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "start_stop_success_recovery",
              "Not recovered", rtn)


def test_primitive_sequence(address):
    print("Testing manipulation primitive sequence...")
    rtn = start_task(address, "TestTask1", queue=False, parameters={"parameters": {"skill_test": 4,
                                                                                   "mp_sequence": [1, 2, 3, 4, 5, 6]}})
    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(
        rtn["result"]["task_result"]["results"]["t1_s1"]["result_graph"] == ["mp_1", "mp_2", "mp_3", "mp_4", "mp_5",
                                                                             "mp_6"], "primmitive_sequence",
        "Wrong primitive sequence", rtn)

    rtn = start_task(address, "TestTask1", queue=False, parameters={"parameters": {"skill_test": 4,
                                                                                   "mp_sequence": [2, 4, 6]}})
    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    msg_error(rtn["result"]["task_result"]["results"]["t1_s1"]["result_graph"] == ["mp_2", "mp_4", "mp_6"],
              "primmitive_sequence",
              "Wrong primitive sequence", rtn)


def test_multilayer_tasks(address):
    print('Testing two-layer task...')
    rtn = start_task(address, "TestTask2", queue=False,
                     parameters={"parameters": {'d': [1, 2], 'e': True, 'success': False,
                                                'stop_level': 4},
                                 "skills": {"t2_s1": {
                                     "skill": {
                                         "exception": "abc"
                                     }
                                 }
                                 },
                                 'subtasks': {'t2_t1': {'parameters': {
                                     'a': [10, 11,
                                           12],
                                     'b': True,
                                     'success': True,
                                     "skill_test": 2},
                                     "skills": {
                                         "t1_s2": {
                                             "skill": {
                                                 "exception": "cba"
                                             }
                                         }
                                     }}}})
    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    results = rtn["result"]["task_result"]["results"]
    msg_error(results["d"] == [1, 2], 'two_layer_task', 'Parameter throughput is faulty.', rtn)
    msg_error(results["e"] == True, 'two_layer_task', 'Parameter throughput is faulty.', rtn)
    msg_error(results["t2_s1"]["exception"] == "abc", 'two_layer_task', 'Parameter throughput is faulty.', rtn)
    msg_error(results["t2_t1"]["t1_s2"]["exception"] == "cba", 'two_layer_task', 'Parameter throughput is faulty.', rtn)

    print('Testing three-layer task...')
    rtn = start_task(address, "TestTask3", queue=False,
                     parameters={'parameters': {'g': [20, 21, 22, 23], 'h': False, 'i': 98.33,
                                                'j': 'this is j', 'stop_level': 3,
                                                'success': True, },
                                 'subtasks': {'t3_t1': {'parameters': {'d': [1, 2],
                                                                       'e': True,
                                                                       'success': False,
                                                                       'stop_level': 4},
                                                        'subtasks': {'t2_t1': {'parameters': {
                                                            'a': [30, 31,
                                                                  32],
                                                            'b': True,
                                                            'success': True}}}}}})
    rtn = wait_for_task(address, rtn['result']['task_uuid'])
    results = rtn["result"]["task_result"]["results"]
    msg_error(results["g"] == [20, 21, 22, 23], 'three_layer_task', 'Parameter throughput is faulty.', rtn)
    msg_error(results["t3_t1"]["t2_t1"]["a"] == [30, 31, 32], 'three_layer_task', 'Parameter throughput is faulty.',
              rtn)


def test_subtask_start_stop(address):
    print('Testing exceptional stop...')
    # input('Press Enter to continue...')
    rtn = start_task(address, "TestTask3", queue=True)
    time.sleep(1)
    stop_task(address, raise_exception=True)
    rtn = wait_for_task(address, rtn["result"]["task_uuid"])

    msg_error(rtn['result']['task_result']["exception"] is True, 'subtask_start_stop_exception',
              'Result has no exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'subtask_start_stop_exception',
              'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "subtask_start_stop_exception",
              "Has recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "start_stop_exception_recovery",
              "No external stop", rtn)

    print('Testing exceptional stop with recovery...')
    # input('Press Enter to continue...')
    rtn = start_task(address, "TestTask3", queue=True)
    time.sleep(1)
    stop_task(address, raise_exception=True, recover=True)
    rtn = wait_for_task(address, rtn["result"]["task_uuid"])

    msg_error(rtn['result']['task_result']["exception"] is True, 'subtask_start_stop_success_exception',
              'Result has no exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'subtask_start_stop_success_exception',
              'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "subtask_start_stop_success_exception",
              "Has recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "subtask_start_stop_success_exception",
              "No external stop", rtn)

    print('Testing stop without recovery...')
    # input('Press Enter to continue...')
    rtn = start_task(address, "TestTask3", queue=True)
    time.sleep(1)
    stop_task(address)
    rtn = wait_for_task(address, rtn["result"]["task_uuid"])
    msg_error(rtn['result']['task_result']["exception"] is False, 'subtask_start_stop_fail',
              'Result has exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'subtask_start_stop_fail',
              'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is False, "subtask_start_stop_fail",
              "Has recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "subtask_start_stop_fail",
              "No external stop", rtn)

    print('Testing stop with recovery...')
    # input('Press Enter to continue...')
    rtn = start_task(address, "TestTask3", queue=True)
    time.sleep(1)
    stop_task(address, recover=True)
    rtn = wait_for_task(address, rtn["result"]["task_uuid"])
    msg_error(rtn['result']['task_result']["exception"] is False, 'subtask_start_stop_fail_recovery',
              'Result has exception',
              rtn)
    msg_error(rtn['result']['task_result']["success"] is False, 'subtask_start_stop_fail_recovery',
              'Result is successful',
              rtn)
    msg_error(rtn["result"]["task_result"]["results"]["recovered"] is True, "subtask_start_stop_fail_recovery",
              "Has not recovered", rtn)
    msg_error(rtn["result"]["task_result"]["external_stop"] is True, "subtask_start_stop_fail_recovery",
              "No external stop", rtn)


def test_memory(address):
    url = "http://" + address + ":8383"
    # print("Testing task context download...")
    # for i in range(100):
    #     response = call_method(address, 12000, method="download_task_context", payload={"task": "TestTask1"})
    #     print(response)
    #     msg_error(response is not None, "memory_task_download", "Response is none", response)
    #     msg_error(response["result"]["result"] is True, "memory_task_download", "Could not load task.", response)
    #     msg_error(response["result"]["context"]["parameters"]["a"] == [0, 0, 0], "memory_task_download",
    #               "Task description is faulty.", response)

    print("Testing skill context download...")
    for i in range(100):
        response = call_method(address, 12000, method="download_skill_context", payload={"skill": "TestSkill1"})
        msg_error(response is not None, "memory_skill_download", "Response is none", response)
        msg_error(response["result"]["result"] is True, "memory_skill_download", "Could not load skill.", response)
        msg_error(response["result"]["context"]["success"] is None, "memory_skill_download",
                  "Skill description is faulty.", response)

    print("Testing object context download...")
    for i in range(100):
        response = call_method(address, 12000, method="download_object_context", payload={"object": "TestObject1"})
        msg_error(response is not None, "memory_object_download", "Response is none", response)
        msg_error(response["result"]["result"] is True, "memory_object_download", "Could not load object.", response)
        msg_error(response["result"]["context"]["grasp_force"] == 1, "memory_object_download",
                  "Object description is faulty.", response)

    print("Testing object teaching...")

    response = call_method(address, 12000, method="set_object",
                           payload={"object": "TestObject1", "q": [0, 1, 2, 3, 4, 5, 6]})
    msg_error(response["result"]["result"] is True, "memory_object_update", "Could not update object.", response)
    response = call_method(address, 12000, method="download_object_context", payload={"object": "TestObject1"})
    msg_error(response["result"]["result"] is True, "memory_object_update", "Could not load object.", response)
    msg_error(response["result"]["context"]["q"] == [0, 1, 2, 3, 4, 5, 6], "memory_object_update",
              "Object description is faulty.", response)
    response = call_method(address, 12000, method="set_object",
                           payload={"object": "TestObject1", "q": [0, 0, 0, 0, 0, 0, 0]})
    msg_error(response["result"]["result"] is True, "memory_object_update", "Could not update object.", response)
    response = call_method(address, 12000, method="download_object_context", payload={"object": "TestObject1"})
    msg_error(response["result"]["result"] is True, "memory_object_update", "Could not load object.", response)
    msg_error(response["result"]["context"]["q"] == [0, 0, 0, 0, 0, 0, 0], "memory_object_update",
              "Object description is faulty.", response)


def test_task_queue(address):
    url = "http://" + address + ":8383"
    print('Testing queued exceptional stop...')
    # input('Press Enter to continue...')
    results = []
    for i in range(10):
        results.append(start_task(address, "TestTask1", {"parameters": {"queue_number": i}}, queue=True))

    call_method(address, 12000, 'remove_task', {'task_uuid': results[0]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[6]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[9]['result']['task_uuid']})
    input("stop task")
    stop_task(address, True)
    rtn = wait_for_task(address, results[8]['result']['task_uuid'])
    msg_error(rtn["result"]["result"] is False, "queue_exception",
              "Result is not False", rtn)
    wait_for_task(address, results[9]['result']['task_uuid'])
    msg_error(rtn["result"]["result"] is False, "queue_exception",
              "Result is not False", rtn)

    print('Testing queued stop...')
    # input('Press Enter to continue...')
    results = []
    for i in range(10):
        results.append(start_task(address, "TestTask1", {"parameters": {"queue_number": i}}, queue=True))

    call_method(address, 12000, 'remove_task', {'task_uuid': results[0]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[6]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[9]['result']['task_uuid']})
    input("stop task")
    stop_task(address)
    for i in range(10):
        rtn = wait_for_task(address, results[i]['result']['task_uuid'])
        if i == 0:
            msg_error(rtn["result"]["task_result"]["external_stop"] is True, "queue",
                      "No external stop", rtn)
        elif i == 6 or i == 9:
            msg_error(rtn["result"]["result"] is False, "queue",
                      "Result is not False", rtn)
        else:
            msg_error(rtn["result"]["task_result"]["results"]["queue_number"] == i, "queue",
                      "Wrong queue number", rtn)

    print('Testing queued stop with recovery...')
    # input('Press Enter to continue...')
    results = []
    for i in range(10):
        results.append(start_task(address, "TestTask1", {"parameters": {"queue_number": i}}, queue=True))

    call_method(address, 12000, 'remove_task', {'task_uuid': results[0]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[6]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[9]['result']['task_uuid']})
    stop_task(address, recover=True)
    for i in range(10):
        rtn = wait_for_task(address, results[i]['result']['task_uuid'])
        if i == 0:
            msg_error(rtn["result"]["task_result"]["external_stop"] is True, "queue_recovery",
                      "No external stop", rtn)
            msg_error(rtn["result"]["task_result"]["results"]["recovered"] is True, "queue_recovery",
                      "Has not recovered", rtn)
        elif i == 6 or i == 9:
            msg_error(rtn["result"]["result"] is False, "queue_recovery",
                      "Result is not False", rtn)
        else:
            msg_error(rtn["result"]["task_result"]["results"]["queue_number"] == i, "queue_recovery",
                      "Wrong queue number", rtn)

    print('Testing queued exceptional stop and empty queue...')
    # input('Press Enter to continue...')
    results = []
    for i in range(10):
        results.append(start_task(address, "TestTask1", {"parameters": {"queue_number": i}}, queue=True))

    call_method(address, 12000, 'remove_task', {'task_uuid': results[0]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[6]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[9]['result']['task_uuid']})
    stop_task(address, raise_exception=True, empty_queue=True)
    for i in range(10):
        rtn = wait_for_task(address, results[i]['result']['task_uuid'])
        if i == 0:
            msg_error(rtn["result"]["task_result"]["external_stop"] is True, "queue_exception_empty",
                      "No external stop", rtn)
        else:
            msg_error(rtn["result"]["result"] is False, "queue_exception_empty",
                      "Result is not False", rtn)

    print('Testing queued stop and empty queue...')
    # input('Press Enter to continue...')
    results = []
    for i in range(10):
        results.append(start_task(address, "TestTask1", {"parameters": {"queue_number": i}}, queue=True))

    call_method(address, 12000, 'remove_task', {'task_uuid': results[0]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[6]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[9]['result']['task_uuid']})
    stop_task(address, empty_queue=True)
    for i in range(10):
        rtn = wait_for_task(address, results[i]['result']['task_uuid'])
        if i == 0:
            msg_error(rtn["result"]["task_result"]["external_stop"] is True, "queue_empty",
                      "No external stop", rtn)
        else:
            msg_error(rtn["result"]["result"] is False, "queue_empty",
                      "Result is not False", rtn)

    print('Testing queued stop with recovery and empty queue...')
    # input('Press Enter to continue...')
    results = []
    for i in range(10):
        results.append(start_task(address, "TestTask1", {"parameters": {"queue_number": i}}, queue=True))

    call_method(address, 12000, 'remove_task', {'task_uuid': results[0]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[6]['result']['task_uuid']})
    call_method(address, 12000, 'remove_task', {'task_uuid': results[9]['result']['task_uuid']})
    stop_task(address, recover=True, empty_queue=True)
    for i in range(10):
        rtn = wait_for_task(address, results[i]['result']['task_uuid'])
        if i == 0:
            msg_error(rtn["result"]["task_result"]["external_stop"] is True, "queue_recovery_empty",
                      "No external stop", rtn)
            msg_error(rtn["result"]["task_result"]["results"]["recovered"] is True, "queue_recovery_empty",
                      "Has not recovered", rtn)
        else:
            msg_error(rtn["result"]["result"] is False, "queue_recovery_empty",
                      "Result is not False", rtn)


def test_skill_parallel(address):
    print('Testing parallel thread...')
    response = start_task(address, "TestTask1", queue=True, parameters={'parameters': {'skill_test': 2}})
    response = wait_for_task(address, response['result']['task_uuid'])
    msg_error(response is not None, 'live_parameter_server', 'None returned', response)
    task_result = response["result"]["task_result"]
    msg_error("parallels_cnt" in task_result["results"]["t1_s2"], 'parallels_thread',
              'Parallels counter not in results', response)
    msg_error(task_result["results"]["t1_s2"]["parallels_cnt"] > 290, 'parallels_thread',
              'Parallels counter is wrong.', response)


def test_live_parameters(address):
    print("Testing live parameter server...")
    response = start_task(address, "TestTask1", queue=True, parameters={'parameters': {'skill_test': 2}})
    time.sleep(1)
    response_set1 = call_method(address, 12000, "set_live_parameter", {"key": "test_parameter_1", "value": 42})
    msg_error(response is not None, 'live_parameter_server', 'None returned', response_set1)
    response_set2 = call_method(address, 12000, "set_live_parameter",
                                {"key": "test_parameter_2", "value": "hello world"})
    msg_error(response is not None, 'live_parameter_server', 'None returned', response_set2)
    response_set3 = call_method(address, 12000, "set_live_parameter",
                                {"key": "test_parameter_3", "value": [1, 2, 3, 4, 5]})
    msg_error(response is not None, 'live_parameter_server', 'None returned', response_set3)
    response = wait_for_task(address, response['result']['task_uuid'])
    msg_error(response is not None, 'live_parameter_server', 'None returned', response)
    msg_error("result" in response, 'live_parameter_server', 'None returned', response)
    task_result = response["result"]["task_result"]
    msg_error("test_parameter_1" in task_result["results"]["t1_s2"], 'live_parameter_server',
              'Test parameter not in results', response)
    msg_error("test_parameter_2" in task_result["results"]["t1_s2"], 'live_parameter_server',
              'Test parameter not in results', response)
    msg_error("test_parameter_3" in task_result["results"]["t1_s2"], 'live_parameter_server',
              'Test parameter not in results', response)
    msg_error(task_result["results"]["t1_s2"]["test_parameter_1"] == 42, 'live_parameter_server',
              'test_parameter_1 has not been put through.', response)
    msg_error(task_result["results"]["t1_s2"]["test_parameter_2"] == "hello world", 'live_parameter_server',
              'test_parameter_2 has not been put through.', response)
    msg_error(task_result["results"]["t1_s2"]["test_parameter_3"] == [1, 2, 3, 4, 5], 'live_parameter_server',
              'test_parameter_3 has not been put through.', response)


def test_controllers(address):
    # print("Testing controller pipeline: 2")
    # T_EE_g = [np.array(
    #     [0.978038, -0.20829, 0.00753134, 0, -0.207918, -0.97754, -0.0344342, 0, 0.0145345, 0.0321121, -0.999379, 0,
    #      0.207653, 0.452514, 0.510002, 1]),
    #     np.array(
    #         [0.710559, 0.394384, 0.582724, 0, -0.546773, -0.211783, 0.810054, 0, 0.442883, -0.894209, 0.0651547,
    #          0, 0.477753, 0.0348045, 0.548409, 1])]
    # for T_EE in T_EE_g:
    #     response = start_task(address, "MoveToCartPose", parameters={"parameters": {"T_EE_g": T_EE.tolist(),
    #                                                                                 "speed": [1, 1], "acc": [1, 1]}})
    #     print(response)
    #     response = wait_for_task(address, response['result']['task_uuid'])
    #     response = call_method(address, 12000, "get_state")
    #     T_EE_now = np.asarray(response["result"]["O_T_EE"])
    #     T_EE_diff = np.linalg.norm(T_EE[12:15] - T_EE_now[12:15])
    #     msg_error(float(T_EE_diff) < 0.01, "test_controllers", "Goal pose and actual pose are too far apart.", response)

    print("Testing controller pipeline: 3")
    q_g = [np.array([0.303584, -0.648212, 0.649549, -2.15934, 0.418512, 1.53807, 0.766482]),
           np.array([-0.195319, -0.447499, 0.0966129, -2.34595, -0.247264, 2.01693, 0.456673])]
    for q in q_g:
        response = start_task(address, "MoveToJointPose", parameters={"parameters": {"q_g": q.tolist()}})
        response = wait_for_task(address, response['result']['task_uuid'])
        response = call_method(address, 12000, "get_state")
        q_now = np.asarray(response["result"]["q"])
        q_diff = np.linalg.norm(q - q_now)
        msg_error(float(q_diff) < 0.1, "test_controllers", "Goal pose and actual pose are too far apart.", response)
        input("Press key")


def test_generic_task(address):
    response = start_task(address, "GenericTask", parameters={"parameters": {
        "skill_names": ["move"],
        "skill_types": ["MoveToContact"]
    },
        "skills": {
            "move": {
                "skill": {
                    "speed": 0.5,
                    "objects": {
                        "goal_pose": "table"
                    }
                },
                "control": {
                    "control_mode": 2
                }
            }
        }})
    print(response)


def test_logging(address):
    print('Testing exceptional stop...')
    rtn = start_task(address, "TestTask1", queue=True, parameters={
        "skills": {
            "t1_s1": {
                "skill": {
                    "log_data": True,
                    "data_length": 2000
                }
            }
        }
    })
    msg_error(rtn['result']['result'], 'start_stop_exception', 'Result is false', rtn)
    task_uuid = rtn["result"]["task_uuid"]
    time.sleep(1)
    rtn = stop_task(address, True)
    msg_error(rtn is not None, 'start_stop_exception', 'None returned', rtn)
    msg_error(rtn['result']['result'], 'start_stop_exception', 'Result is false', rtn)
    rtn = wait_for_task(address, task_uuid)


def test_telemetry_udp(address: str, subscriber_addr: str, subscriber_port: int = 12346):
    print("Testing Telemtry_UDP module...")
    print("subscribe to Telemetry_UDP with \"tau_ext\", \"q\"...")
    result_1 = call_method(address, 12000, "subscribe_telemetry",
                           {"ip": subscriber_addr, "port": subscriber_port, "subscribe": ["tau_ext", "q"]},silent=False, timeout=7)
    if result_1["result"]["result"]:
        print("successfull subscribed.")
    else:
        print("Error while subscribing: ", result_1)
    print("receiving subscribed telemetry packages for next 10 seconds...")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((subscriber_addr, subscriber_port))
    cnt = 0
    received_pkgs = []
    start_time = time.time()
    try:
        print("\n    --Interrupt with ctrl+c--\n")
        while True:
            data, adrr = s.recvfrom(8192)
            received_pkgs.append(json.loads(data.decode("utf-8")))
            cnt += 1
            if time.time() - start_time > 10:
                break
    except KeyboardInterrupt:
        pass
    end_time = time.time()
    print("received ", cnt, " packages over last", end_time - start_time, " seconds")
    pkg_validation_cnt = 0
    for pkg in received_pkgs:
        if pkg.get("tau_ext", False) != False and pkg.get("q", False) != False:
            pkg_validation_cnt += 1
    print(cnt - pkg_validation_cnt, " package(s) are corrupted.")
    print("unsubscribe...")
    result_2 = call_method(address, 12000, "unsubscribe_telemetry", {"ip": subscriber_addr})
    if result_2["result"]["result"]:
        print("successfull unsibscribed.")
    if cnt < 1:
        print("Received no package. Test failed!")
    elif result_1["result"]["result"] and cnt - pkg_validation_cnt == 0 and result_2["result"]["result"]:
        print("\nEverything works fine :)")
    else:
        print("\nTest failed!")


def start_skill(address: str, skill: str, parameters: dict, control: dict):
    response = start_task(address, "GenericTask", parameters={"parameters": {
        "skill_names": ["skill"],
        "skill_types": [skill]
    },
        "skills": {
            "skill": {
                "skill": parameters,
                "control": control
            }
        }})


def gripper_tests(url):
    # grasp test object, frame check
    # release object, frame check
    # home gripper
    pass


def desk_tests(url):
    # execute timeline and stop
    pass


def general_function_tests(url):
    # lock/unlock brakes
    # pack pose
    # shutdown
    pass


def info_function_tests(url):
    # state check
    # ip check
    pass

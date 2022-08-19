import requests
import json
import websockets
import asyncio


def call_method(hostname: str, port: int, method: str, payload=None, timeout: float=1000):
    headers = {'content-type': 'application/json'}

    message = {
        "id": 1,
        "jsonrpc": "2.0",
        "method": method,
        "params": payload,
    }
    url = "http://" + hostname + ":" + str(port)
    try:
        response = requests.post(url, data=json.dumps(message), headers=headers, timeout=timeout).json()
    except requests.Timeout:
        print('Timeout, server has terminated or does not exist.')
        response = None
    except requests.ConnectionError:
        print("Connection error, target url " + url + " not reachable.")
        response = None

    return response


def start_task(hostname: str, task: str, parameters={}, queue=False, timeout: float=1000):
    payload = {
        'task': task,
        'queue': queue,
        'parameters': parameters
    }
    r = call_method(hostname, 12001, 'start_task', payload, timeout)
    return r


def wait_for_task(hostname: str, task_uuid: str):
    payload = {
        'task_uuid': task_uuid
    }
    input('Press')
    r = call_method(hostname, 12001, 'wait_for_task', payload)
    print(r)


def stop_task(hostname: str, raise_exception=False, recover=False, empty_queue=False):
    params={
        'raise_exception': raise_exception,
        'recover': recover,
        "empty_queue": empty_queue
    }
    r = call_method(hostname, 12001, 'stop_task', params)
    return r

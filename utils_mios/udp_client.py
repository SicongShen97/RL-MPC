import socket
import json
import time


def call_method(hostname: str, port: int, method: str, payload=None, timeout: float = -1) -> dict:
    request = {
        "method": method,
        "request": payload
    }
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(request).encode('utf-8'), (hostname, port))
    if timeout >= 0:
        s.settimeout(timeout)
    response = {"result": {"result": False, "error": "No response"}}
    try:
        data, address = s.recvfrom(8192)
        if address[1] != port:
            response = {"result": {"result": False, "error": "Sender has different address then receiver!"}}
        else:
            response = json.loads(data.decode("utf-8"))
    except socket.timeout:
        response = {"result": {"result": False, "error": "Connection timed out"}}
    return response


def start_task(hostname: str, task: str, parameters={}, queue=False, timeout=-1) -> dict:
    payload = {
        "task": task,
        "parameters": parameters,
        "queue": queue
    }
    return call_method(hostname, 12002, "start_task", payload, timeout)


def stop_task(hostname: str, raise_exception=False, recover=False, empty_queue=False) -> dict:
    payload = {
        "raise_exception": raise_exception,
        "recover": recover,
        "empty_queue": empty_queue
    }
    return call_method(hostname, 12002, "stop_task", payload)


def wait_for_task(hostname: str, task_uuid: str) -> dict:
    payload = {
        "task_uuid": task_uuid
    }
    return call_method(hostname, 12002, "wait_for_task", payload)


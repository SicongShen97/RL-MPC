import rospy
import json
from mios_msg.srv import *


def start_task(hostname: str, task: str, parameters={}, queue=False):
    rospy.wait_for_service("start_task")
    try:
        srv = rospy.ServiceProxy("start_task", StartTask)
        response = srv(task, json.dumps(parameters), queue)
        return response.task_uuid
    except rospy.ServiceException as e:
        print("Service call failed: " + str(e))

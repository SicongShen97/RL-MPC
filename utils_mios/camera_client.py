import pickle
import time

import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

for request in range(10):
    print("Sending request {} …".format(request))
    socket.send(b"Hello")
    message = socket.recv()
    pca_data = pickle.loads(message)
    # print("Received reply {}[{}]".format(request, message))
    print("Received reply {} mean value [{}]".format(request, pca_data["M"][0]))
    time.sleep(3)

socket.close()
context.term()
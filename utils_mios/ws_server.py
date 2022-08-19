from typing import Optional, Awaitable

from tornado import websocket
import tornado.ioloop
import json


class WSServer(websocket.WebSocketHandler):

    def data_received(self, chunk: bytes) -> Optional[Awaitable[None]]:
        pass

    def __init__(self, port, endpoint="default"):

        self.methods = dict()
        self.bind_method("test_fun", self.test_fun)

        #self.application = tornado.web.Application([(r"/" + endpoint, WSServer), ])
        #self.application.listen(port)
        #super(WSServer, self).__init__(self.application)
    def _initialize(self):
        self.channel = None

    def open(self, channel):
        print("Websocket Opened")
        self.channel = channel

    def method(self):
        print("method")

    def on_message(self, message):
        print("TEST")
        print(message)
        self.write_message(u"You said: %s" % message)
        message_json = json.loads(message)
        if "method" not in message_json:
            print("Message does not contain a method name.")
            return None
        method = message_json["method"]
        if method not in self.methods:
            print("Unknown method: " + method)
            return None
        if "request" not in message_json:
            request = {}
        else:
            request = message_json["request"]
        return self.methods[method](request)

    def on_close(self):
        print("Websocket closed")

    def bind_method(self, name, fun):
        if name in self.methods:
            print("Method with name " + name + " already exists.")
            return False
        self.methods[name] = fun

    def test_fun(self, request2):
        print(request2)
        return 0


if __name__ == "__main__":
    application = tornado.web.Application([(r"/", WSServer), ])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9000, "localhost")

    # Start IO/Event loop
    print("Starting web server")
    tornado.ioloop.IOLoop.instance().start()

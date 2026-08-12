"""
TODO
- FIX CATASTROPHIC CRASH WHEN DISCONNECTING FROM INTERFACE SIDE
- Force async server, avoid lag during debug
"""

from websockets.sync.server import serve
import websockets
import json
import threading
import cv2
import base64
import socket

class WSServerHandlerID:
    def __init__(self, message, index):
        self.message = message
        self.index = index

class WSServer:
    def __init__(self, host='0.0.0.0', port=8765):
        self.host = host
        self.port = port
        self.clients = []
        self.registered = False

        self.handlers = {}

    def add_handler(self, message, callback):
        if self.handlers.get(message) is None:
            self.handlers[message] = []
        self.handlers[message].append(callback)
        return WSServerHandlerID(message, len(self.handlers[message]) - 1)

    def remove_handler(self, handler_id):
        if type(handler_id) is not WSServerHandlerID:
            raise ValueError("handler_id must be of type WSServerHandlerID. Ensure you are passing the return value of add_handler() to remove_handler().")
        if self.handlers.get(handler_id.message) is None:
            return False
        if handler_id.index >= len(self.handlers[handler_id.message]):
            return False
        self.handlers[handler_id.message].pop(handler_id.index)
        return True

    def clear_handlers(self, message=None):
        if message is None:
            self.handlers = {}
        else:
            if self.handlers.get(message) is not None:
                self.handlers[message] = []

    def register_client(self, websocket):
        self.clients.append(websocket)
        print(f"Client connected. Total clients: {len(self.clients)}")

    def unregister_client(self, websocket):
        self.clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(self.clients)}")

    def broadcast(self, message):
        if self.clients:
            for client in self.clients:
                client.send(json.dumps(message))

    def send_frame(self, frame):
        ret, buffer = cv2.imencode('.jpg', frame)

        if ret:
            frame_base64 = base64.b64encode(buffer).decode('utf-8')
            self.broadcast({"message": "image", "data": frame_base64})

    def handle_message(self, websocket, message):
        try:
            data = json.loads(message)
            # Check connection is valid
            if not self.registered:
                if data.get("message") == "register":
                    self.registered = True
                    print("Client registered successfully.")
                else:
                    websocket.send(json.dumps({"message":"error", "error": "Client not registered. Please send a 'register' message first."}))
                    return
            # Call relevant handlers
            if data.get("message") and self.handlers.get(data["message"]):
                for callback in self.handlers[data["message"]]:
                    callback(websocket, data.get("data", {}))

        except json.JSONDecodeError:
            websocket.send(json.dumps({"message":"error", "error": "Invalid JSON"}))

    def client_handler(self, websocket):
        self.register_client(websocket)
        try:
            for message in websocket:
                self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.unregister_client(websocket)

    def _start_server(self):
        print(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        with serve(self.client_handler, self.host, self.port) as server:
            server.serve_forever()


    def run(self):
        self.run_thread = threading.Thread(target=self._start_server, daemon=True)
        self.run_thread.start()

if __name__ == "__main__":
    import cv2
    import base64
    import numpy as np

    server = WSServer()
    server.run()
    print("Running server, starting camera...")

    # Add handlers
    def move_handler(websocket, data):
        x = data.get("x", 0)
        y = data.get("y", 0)
        rot = data.get("rotation", 0)

        move_dir = np.arctan2(x, y) * 180 / np.pi
        move_mag = min(np.hypot(x, y), 1)
        move_rot = rot

        print(f"Move command: dir={move_dir}, mag={move_mag}, rot={move_rot}")

    server.add_handler("move", move_handler)

    # Start camera
    camera = cv2.VideoCapture(0)

    # Set camera resolution to lower values for better performance
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)

    while 1:
        cv2.waitKey(1)
        ret, frame = camera.read()

        if ret:
            server.send_frame(frame)

        # cv2.imshow("interface test", frame)
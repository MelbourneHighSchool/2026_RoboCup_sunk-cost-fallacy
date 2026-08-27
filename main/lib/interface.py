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
import queue

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

        self.send_buffer = queue.Queue()

    def _server_loop(self):
        print(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        with serve(self.client_handler, self.host, self.port) as server:
            server.serve_forever()

    def _send_loop(self):
        while True:
            if not self.send_buffer.empty():
                message = self.send_buffer.get()
                if type(message) is not dict:
                    # Encode as image
                    ret, buffer = cv2.imencode('.jpg', message)
                    if ret:
                        frame_base64 = base64.b64encode(buffer).decode('utf-8')
                        self._broadcast({"message": "image", "data": frame_base64})
                else:
                    self._broadcast(message)

    def run(self):
        self.run_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)

        self.run_thread.start()
        self.send_thread.start()

    # HANDLERS

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

    # BOT HANDLING

    def register_client(self, websocket):
        self.clients.append(websocket)

    def unregister_client(self, websocket):
        self.clients.remove(websocket)

    def client_handler(self, websocket):
            self.register_client(websocket)
            try:
                for message in websocket:
                    self.handle_message(websocket, message)
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.unregister_client(websocket)

    # MESSAGE HANDLING

    def _broadcast(self, message):
        if self.clients:
            for client in self.clients:
                client.send(json.dumps(message))

    def send_frame(self, message):
        """Send a frame or message to all connected clients."""
        self.send_buffer.put(message)
        return True

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
import cv2
import json
import websocket
import base64
import numpy as np


ws = websocket.WebSocket()
host = "localhost"  # Default host

while True:
    cv2.destroyAllWindows()
    x = input(f"Input host (current: {host}): ")
    if x:
        host = x
    try:
        ws.connect(f"ws://{host}:8765", timeout=5)
    except ConnectionRefusedError:
        print(f"Failed to connect to ws://{host}:8765. Please check the host and try again.")
        continue
    ws.send('{"message": "register"}')

    try:
        while True:
            try:
                data = ws.recv()
            except websocket.WebSocketTimeoutException:
                print("Connection timed out. Attempting to reconnect...")
                ws.close()
                break
            except websocket.WebSocketPayloadException:
                print("Received invalid payload. Attempting to reconnect...")
                ws.close()
                break
            if data:
                message = json.loads(data)
                if message.get("message") == "image":
                    frame_base64 = message.get("data")
                    frame_bytes = base64.b64decode(frame_base64)
                    frame_array = np.frombuffer(frame_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                    # Display the frame
                    cv2.imshow("Received Frame", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
    except websocket.WebSocketConnectionClosedException:
        print("Connection closed.")
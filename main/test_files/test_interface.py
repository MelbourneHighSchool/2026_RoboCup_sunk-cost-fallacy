"""Test the interface by just streaming camera frames (no drawing)"""
from lib.interface import WSServer
from lib.Vision import Vision

def main(config):
    server = WSServer()
    server.run()

    vision = Vision()
    vision.start()

    while True:
        frame = vision.camera.latest_frame
        server.send_frame(frame)
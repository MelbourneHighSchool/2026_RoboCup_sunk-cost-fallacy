from lib.config import Config

from enum import IntEnum
from multiprocessing import Process, Value, Queue
from queue import Full, Empty
from ctypes import c_bool

import socket
import threading
import numpy as np
import cv2

HOST = "0.0.0.0"
PORT = 8765
BT_CHANNEL = 17

class CommMode(IntEnum):
    SERVER = 0
    CLIENT = 1

class Communication:
    def __init__(self, config:Config, bot_comm_en:bool=True, remote_comm_en:bool=False, timeout=5):
        # flags
        self.bot_comm_en = bot_comm_en
        self.remote_comm_en = remote_comm_en

        self.bot_connected_v = Value(c_bool, False)
        self.remote_connected_v = Value(c_bool, False)

        # bot comm
        comm_settings = config.get_value("comm", {})

        bot_mode = comm_settings.get("mode", None)
        bot_mac = comm_settings.get("mac_addr", None)

        if bot_comm_en and not bot_mac:
            raise ValueError("MAC address must be provided in the configuration for bot communication.")

        if bot_comm_en and bot_mode is None:
            raise ValueError("Communication mode must be provided in the configuration for bot communication.")

        # setup message queues
        self.bot_recv_q = Queue(16)
        self.bot_send_q = Queue(16)

        self.rem_recv_q = Queue(16)
        self.rem_send_q = Queue(16)

        # spawn processes for bot and remote communication
        if bot_comm_en:
            self.bot_proc = Process(target=self._bot_loop, args=(
                bot_mac, BT_CHANNEL,
                self.bot_recv_q, self.bot_send_q,
                self.bot_connected_v, bot_mode,
                timeout,
                self._bot_listen
            ))

            self.bot_proc.start()

        if remote_comm_en:
            self.rem_proc = Process(target=self._rem_loop, args=(
                HOST, PORT,
                self.rem_recv_q, self.rem_send_q,
                self.remote_connected_v,
                timeout
            ))

            self.rem_proc.start()

    # Raw read/write methods

    def _send_raw_bot(self, data:bytes):
        if self.bot_connected:
            try:
                self.bot_send_q.put(data, block=False)
                return True
            except Full:
                print("Bot send queue is full. Dropping message.")
                return False
        
        return False

    def _read_raw_bot(self):
        if self.bot_connected and not self.bot_recv_q.empty():
            try:
                return self.bot_recv_q.get(block=False)
            except Empty:
                return None
        return None

    def _send_raw_remote(self, data:bytes):
        if self.remote_connected:
            try:
                self.rem_send_q.put(data, block=False)
                return True
            except Full:
                print("Remote send queue is full. Dropping message.")
                return False
        
        return False

    def _read_raw_remote(self):
        if self.remote_connected and not self.rem_recv_q.empty():
            try:
                return self.rem_recv_q.get(block=False)
            except Empty:
                return None
        return None

    # Processed read/write methods

    def send_frame(self, frame:np.ndarray):
        self.send_raw_remote(frame.tobytes())

    @property
    def bot_connected(self):
        return self.bot_connected_v.value

    @property
    def remote_connected(self):
        return self.remote_connected_v.value

    @staticmethod
    def _bot_listen(bot_conn:socket.socket, recv_q:Queue, bot_connected_v):
        while True:
            if bot_connected_v.value and bot_conn:
                try:
                    data = bot_conn.recv(1024)
                    if data:
                        if recv_q.full():
                            recv_q.get_nowait()
                        recv_q.put(data, block=False)
                except (ConnectionResetError, ConnectionAbortedError, socket.timeout):
                    print("Bot connection lost. Waiting for a new connection...")
                    bot_connected_v.value = False
                    bot_conn.close()
            else:
                # kill thread, since the main loop will handle reconnection
                return


    @staticmethod
    def _bot_loop(bot_mac:str, bot_channel:int,
                  recv_q:Queue, send_q:Queue,
                  bot_connected_v, bot_mode:int,
                  timeout:int,
                  listen_method):

        if bot_mode == CommMode.SERVER:
            # setup socket
            bot_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            bot_sock.settimeout(timeout)
            try:
                bot_sock.bind((bot_mac, bot_channel))
            except Exception as e:
                print(f"Failed to bind to {bot_mac}:{bot_channel}. Error: {e}. Quitting bot communication loop.")
                return
            bot_sock.listen(1)
            # main loop
            bot_conn, bot_addr = None, None
            listen_thread = None
            while True:
                if bot_connected_v.value:
                    try:
                        # send stuff
                        if not send_q.empty():
                            data = send_q.get(block=False)
                            bot_conn.sendall(data)
                    except (ConnectionResetError, ConnectionAbortedError, socket.timeout) as e:
                        print(f"Bot connection lost. Waiting for a new connection... {e}")
                        bot_connected_v.value = False
                        bot_conn.close()
                else:
                    # receive a new connection
                    try:
                        print(f"Waiting for a new bot connection on {bot_mac}:{bot_channel}...")
                        bot_conn, bot_addr = bot_sock.accept()
                    except socket.timeout:
                        continue
                    bot_connected_v.value = True
                    listen_thread = threading.Thread(target=listen_method, args=(bot_conn, recv_q, bot_connected_v), daemon=True)
                    listen_thread.start()

        else:
            ...




    @staticmethod
    def _rem_loop(rem_host:str, rem_port:int,
                  recv_q:Queue, send_q:Queue,
                  remote_connected_v,
                  timeout:int):

        ...

    @staticmethod
    def _comm_proc(
        bot_comm_en:bool, remote_comm_en:bool,
        bot_mac:str, bot_channel:int, comm_mode:int,
        rem_host:str, rem_port:int,
        timeout:int,

        bot_connected_v, remote_connected_v,

        bot_loop, rem_loop,

        bot_recv_q, bot_send_q,
        rem_recv_q, rem_send_q
    ):
        # init stuff
        bot_sock = None
        bot_thread = None

        rem_sock = None
        rem_thread = None

        # bot comm (inter-bot communication)
        if bot_comm_en:
            # setup socket
            bot_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            bot_sock.settimeout(timeout)

            # setup thread
            bot_thread = threading.Thread(target=bot_loop, args=(bot_sock, bot_mac, bot_channel,
                                                                 bot_recv_q, bot_send_q,
                                                                 bot_connected_v, comm_mode), daemon=True)
            bot_thread.start()

        # remote comm (bot to laptop communication, debug)
        if remote_comm_en:
            # setup socket
            rem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rem_sock.settimeout(timeout)

            # setup thread
            rem_thread = threading.Thread(target=rem_loop, args=(rem_sock, rem_host, rem_port,
                                                                 rem_recv_q, rem_send_q,
                                                                 remote_connected_v), daemon=True)
            rem_thread.start()

        if bot_thread:
            bot_thread.join()

        if rem_thread:
            rem_thread.join()

        return
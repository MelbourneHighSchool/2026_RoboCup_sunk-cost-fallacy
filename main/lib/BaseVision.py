### CURRENTLY WORKING ON: Making it not timeout. See c_new_frame.wait_for in proc_base

### TODO: Plan out and complete enable/disable feature
# Each ProcEntry will have 

import picamera2
from libcamera import controls
from multiprocessing import Process, Value, Array, Condition, Event
from multiprocessing.shared_memory import SharedMemory
import time
import numpy as np
import cv2
import traceback

from multiprocessing.synchronize import Condition as Condition_T, Event as Event_T
from multiprocessing.sharedctypes import Synchronized as Synchronized_T
from ctypes import c_uint8, c_float, c_bool
from typing import Callable, Any, NamedTuple

class BroadcasterArgs(NamedTuple):
    size: tuple[int, int]
    shm_name: str
    v_latest_idx: Synchronized_T  #[c_uint8]
    v_latest_timestamp: Synchronized_T  #[c_float]
    c_new_frame: Condition_T
    e_exit: Event_T
    next_frame_timeout: float

class BaseProcArgs(NamedTuple):
    frame_size: int
    frame_shape: tuple[int, int, int]
    latest_idx: int
    latest_timestamp: float
    frame: cv2.Mat
    enabled: bool

class ProcEntry(NamedTuple):
    init: Callable
    loop: Callable
    deinit: Callable  #=lambda: None
    init_args: tuple  #=()
    v_enabled: Synchronized_T

class Camera:
    def __init__(self, size: tuple[int, int] = (640, 640), shm_name: str = "type shii"):
        # Store args and useful stuff
        self.size = size
        self.shm_name = shm_name
        self.frame_size = size[0] * size[1] * 3
        self.frame_shape = (size[1], size[0], 3)  # Rows, Columns, Components!!!
        self.center = np.array([size[0]>>1, size[1]>>1], dtype=np.int16)

        # Initialise camera for low latency
        self.camera = picamera2.Picamera2()
        cfg = self.camera.video_configuration
        cfg.main.format = "RGB888"
        cfg.raw.size = (2304, 1296)
        cfg.main.size = size
        cfg.buffer_count = 3
        cfg.controls.FrameRate = 56.8

        self.camera.configure(cfg)
        self.camera.pre_callback = self._camera_callback

        # If last run leaked the SharedMemory, clean it up now
        try:
            old_shm = SharedMemory(shm_name, create=False)
            old_shm.close()
            old_shm.unlink()
            print("[BaseVision] Cleaned up leaked frame buffer from last run")
        except FileNotFoundError:
            pass

        # Initialise buffer (a ring buffer of size 3)
        self.frame_buffer = SharedMemory(shm_name, create=True, size=self.frame_size*3)
        self.v_latest_idx = Value(c_uint8, 0)
        self.v_latest_timestamp = Value(c_float, time.monotonic())
        self.c_new_frame = Condition()

        # Stuff to ensure termination
        self.e_exit = Event()
        self.next_frame_timeout = 3.0
    
    def create_broadcaster_args(self, next_frame_timeout = 10.0) -> BroadcasterArgs:
        # Pass these into the Broadcaster class initialiser
        # Isolation is needed between these classes as I can't initialise multiple Picamera2s.
        self.next_frame_timeout = next_frame_timeout
        return BroadcasterArgs(
            self.size,
            self.shm_name,
            self.v_latest_idx,
            self.v_latest_timestamp,
            self.c_new_frame,
            self.e_exit,
            self.next_frame_timeout
        )
    
    def start(self):
        self.camera.start()
        self.camera.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 32.0})
    
    def stop(self, terminate=False):
        self.camera.stop()
        if terminate:
            self.e_exit.set()
    
    @property
    def latest_frame(self):
        latest_idx = self.v_latest_idx.value
        slc = slice(self.frame_size * latest_idx, self.frame_size * (latest_idx + 1))
        return np.reshape(self.frame_buffer.buf[slc], self.frame_shape)
    
    def _camera_callback(self, request):
        new_idx = (self.v_latest_idx.value + 1) % 3
        new_timestamp = time.monotonic()

        # Copy new frame
        with picamera2.MappedArray(request, "main") as m:
            slc = slice(self.frame_size * new_idx, self.frame_size * (new_idx + 1))
            self.frame_buffer.buf[slc] = m.array.ravel()[:]
        
        # Update metadata and notify all readers that a new frame has arrived
        self.v_latest_idx.value = new_idx
        self.v_latest_timestamp.value = new_timestamp
        with self.c_new_frame:
            self.c_new_frame.notify_all()

class Broadcaster:
    def __init__(self, broadcaster_args: BroadcasterArgs):
        self.broadcaster_args = broadcaster_args
        self.procs: dict[str, Process | None] = {}
        self.proc_entries: dict[str, ProcEntry] = {}
    
    def register_proc(self, name: str, init: Callable, loop: Callable, deinit: Callable=lambda _: None, init_args: tuple=()):
        if not deinit: deinit = lambda _: None
        v_enabled = Value(c_bool, True)
        self.proc_entries[name] = ProcEntry(init, loop, deinit, init_args, v_enabled)
    
    def unregister_proc(self, name):
        del self.proc_entries[name]
    
    def unregister_all_procs(self):
        self.proc_entries = {}
    
    def enable_proc(self, name):
        self.proc_entries[name].v_enabled.value = True
    
    def disable_proc(self, name):
        self.proc_entries[name].v_enabled.value = False
    
    def enable_all_procs(self, name):
        for p in self.proc_entries.values():
            p.v_enabled.value = True
    
    def disable_all_procs(self, name):
        for p in self.proc_entries.values():
            p.v_enabled.value = False
    
    def start(self):
        e_exit = self.broadcaster_args.e_exit
        e_exit.clear()

        if self.procs:
            print("[BaseVision] Attempted to start processes when they're already running. Ignoring this reuqest")
            return
        
        # Add registered processes
        for pname, (init, loop, deinit, init_args, v_enabled) in self.proc_entries.items():
            self.procs[pname] = Process(
                name=pname,
                target=self.proc_base,
                args=(pname, init, loop, deinit, init_args)
            )
        
        for p in self.procs.values():
            p.start()
    
    def stop(self, wait: bool=True, timeout: float | None=None):
        e_exit = self.broadcaster_args.e_exit
        e_exit.set()

        with self.broadcaster_args.c_new_frame:
            self.broadcaster_args.c_new_frame.notify_all()

        if not self.procs:
            print("[BaseVision] Attempted to stop processes but none are found. Ignoring this request")
            return

        # wait is False: just don't wait
        if not wait:
            self.procs = {}
            return
        
        # No specified timeout: use default timeout + a little margin of delay
        if timeout is None:
            next_frame_timeout = self.broadcaster_args.next_frame_timeout
            timeout = next_frame_timeout + 1
        
        # timeout negative: wait for all processes to finish
        elif timeout < 0:
            for p in self.procs.values():
                p.join()
            self.procs = {}
            return
        
        # timeout positive: wait with timeout
        start_time = time.monotonic()
        while (time.monotonic() <= start_time + timeout) and any(p.is_alive() for p in self.procs.values()):
            for pname, p in self.procs.items():
                if not p.is_alive():
                    continue

                p.join(timeout=0.2)
                if not p.is_alive():
                    if p.exitcode != 0:
                        print(f"[BaseVision] Process {p.name} exited with non-zero exit code {p.exitcode}")
        
        # Forcefully terminate if processes do not end
        for pname, p in self.procs.items():
            if not p.is_alive():
                continue

            p.terminate()
            p.join()
            print(f"[BaseVision] Process {pname} was forcibly terminated during stop()")
        
        self.procs = {}
    
    def deinit(self, stop_args: tuple[bool, float | None] | None=None):
        if stop_args is not None:
            self.stop(*stop_args)
        else:
            self.stop()
        
        shm_name = self.broadcaster_args.shm_name
        try:
            shm = SharedMemory(shm_name, create=False)
            shm.close()
            shm.unlink()
            print("[BaseVision] Main process successfully cleaned up")
        except FileNotFoundError:
            print("[BaseVision] Main process could not find anything to clean up")
    
    def proc_base(self, pname: str, init: Callable, loop: Callable, deinit: Callable = lambda: None, init_args: tuple=()):
        try:
            # `loop` is the main function that will be repeatedly executed by this base process.
            # It should handle the core processing tasks, excluding any initialization or minimal setup.
            # All processes are expected to extend `proc_base` and define their logic via `loop`.
            #
            # The `loop` function must accept two arguments:
            #   1. `base_args` — information provided by `proc_base` about the current state (e.g., image data)
            #   2. `keep_args` — a shared dictionary between `proc_base` and `loop` for persistent data.
            #
            # If the result of `loop` evaluates to True as a boolean, the process will exit.

            # The `init` function ensures `keep_args` is properly initialized with the correct starting values.
            # It must be a reference type (e.g: dict, list, Value) so that proc_base and loop share it.
            keep_args = init(*init_args)

            # Unpack broadcaster_args and make useful vars
            size = self.broadcaster_args.size
            shm_name = self.broadcaster_args.shm_name
            v_latest_idx = self.broadcaster_args.v_latest_idx
            v_latest_timestamp = self.broadcaster_args.v_latest_timestamp
            c_new_frame = self.broadcaster_args.c_new_frame
            e_exit = self.broadcaster_args.e_exit
            next_frame_timeout = self.broadcaster_args.next_frame_timeout
            v_enabled = self.proc_entries[pname].v_enabled
            
            shm = SharedMemory(shm_name, create=False)
            last_processed_timestamp = float("-inf")
            
            # Useful consts
            frame_size = size[0] * size[1] * 3
            frame_shape = (size[1], size[0], 3)  # Rows, Cols, Components! Reverse order to Picam size parameter

            # Unshared vars holding info about the frame *currently being processed*
            latest_idx = 0
            latest_timestamp = float("-inf")
            frame = np.zeros(frame_shape, dtype=c_uint8)

            # Termination circumstance (for printing termination message)
            # 0: Exit signal
            # 1: Timeout
            # 2: loop() called for exit
            # 3: Error (probably want to separate base error and loop/init error)
            exit_circumstance = 0

            while not e_exit.is_set():  # !!! Add an terminate signal/timeout
                last_processed_timestamp = latest_timestamp
                enabled = v_enabled.value

                # Wait for new frame
                with c_new_frame:
                    res = c_new_frame.wait_for(lambda: (last_processed_timestamp < v_latest_timestamp.value) or (e_exit.is_set()), timeout=next_frame_timeout)
                
                # WARN: Doesn't work
                if not res: # Timeout
                    exit_circumstance = 1
                    break
                
                with v_latest_timestamp.get_lock(): latest_timestamp = v_latest_timestamp.value
                with v_latest_idx.get_lock():       latest_idx = v_latest_idx.value

                # Copy frame while doing a step (hope ring buffer hasn't looped around yet)
                if enabled:
                    slc = slice(frame_size * latest_idx, frame_size * (latest_idx + 1))
                    np.copyto(frame, np.reshape(shm.buf[slc], frame_shape))

                # Prepare args for and run loop function
                base_args = BaseProcArgs(
                    frame_size,
                    frame_shape,
                    latest_idx,
                    latest_timestamp,
                    frame,
                    enabled
                )

                ret = loop(base_args, keep_args)
                if ret:
                    exit_circumstance = 2
                    break
        
        except Exception as e:
            traceback.print_exc()
            exit_circumstance = 3

        # Print exit msg
        match exit_circumstance:
            case 0:
                print(f"[BaseVision] Process {pname} encountered exit signal and is exiting")
            case 1:
                print(f"[BaseVision] Process {pname} timed out and is exiting")
            case 2:
                print(f"[BaseVision] Process {pname}'s loop exited with exit code {ret}, exiting")
            case 3:
                print(f"[BaseVision] Process {pname} encountered an error and is attempting to exit cleanly. Traceback above")
        
        # Cleanup
        try:
            deinit(keep_args)
        except:
            traceback.print_exc()
            print(f"[BaseVision] Process {pname} encountered an error during deinit. Traceback above")
        
        try:
            shm.close()
            print(f"[BaseVision] Process {pname} closed SharedMemory")
        except:
            print(f"[BaseVision] Process {pname} did not find SharedMemory to close")
    
    def proc_example_init(self, *init_args):
        length_of_config = init_args[0]
        v_config = Array(c_bool, 3)  # Create a shared array of 3 bools for config
        v_ok = Value(c_bool, True)  # Create a shared variable so user can check if process is ok
        return (v_config, v_ok)  # Return it so proc_base saves it and feeds it to the loop
    
    def proc_example_loop(self, base_args, keep_args):
        frame_size, frame_shape, latest_idx, latest_timestamp, frame, enabled = base_args  # Unpack base_args
        if not enabled:
            return  # In loop, return something evaluating to False to end (the current iteration) and not raise an error
                    # See "Print exit msg" comment to see what other return values mean
        
        v_config, v_ok = keep_args  # Unpack saved args from init
        v_ok.value = True  # Do something
        return
    
    def proc_example_deinit(self, keep_args):
        ...  # Use this if you have shared memory/gpio pins/etc. you need to deinitialise to safely exit

# Near-minimal example: Report average brightness for 10 seconds
if __name__ == "__main__":
    from multiprocessing import Value
    from multiprocessing.sharedctypes import Synchronized
    from ctypes import c_uint8
    import time

    camera = Camera()
    camera.start()
    broadcaster = Broadcaster(camera.create_broadcaster_args())

    def proc_example_init(v_brightness: Synchronized):  #[c_uint8]
        return v_brightness
    
    def proc_example(base_args: BaseProcArgs, v_brightness: Synchronized):  #[c_uint8]
        frame = base_args.frame
        v_brightness.value = int(frame.mean())
        print(v_brightness.value)
    
    v_brightness = Value(c_uint8, 0)

    broadcaster.register_proc(
        name="proc_example",
        init=proc_example_init,
        loop=proc_example,
        init_args = (v_brightness,)
        # No deinit func for this example
    )

    broadcaster.start()

    # Print brightness of frame for 10s
    for i in range(50):
        brightness = v_brightness.value
        
        print(i, brightness)
        time.sleep(0.1)
    
    # End program
    camera.stop()
    broadcaster.deinit(stop_args=())

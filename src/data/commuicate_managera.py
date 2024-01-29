import os
import pickle
import threading
from uuid import uuid1


class CommuniAgent:
    def __init__(self, name: str):
        self.pub_pipe_path = None
        self.sub_pipe_paths = {}
        self.type = name
        self.id = uuid1()
        self.lock = threading.Lock()

    def init_publisher(self, pub_port):
        # pub_port is actually the path to the named pipe
        self.pub_pipe_path = str(pub_port)
        if not os.path.exists(self.pub_pipe_path):
            os.mkfifo(self.pub_pipe_path)

    def init_subscriber(self, name: str, sub_port):
        # sub_port is actually the path to the named pipe
        sub_pipe_path = str(sub_port)
        if not os.path.exists(sub_pipe_path):
            os.mkfifo(sub_pipe_path)
        self.sub_pipe_paths[name] = sub_pipe_path

    def send_obj(self, data):
        with self.lock:
            try:
                # Open the pipe in non-blocking mode
                fd = os.open(self.pub_pipe_path, os.O_WRONLY | os.O_NONBLOCK)
                with os.fdopen(fd, 'wb') as pipe:
                    pickle.dump(data, pipe)
            except OSError as e:
                # ENXIO: No such device or address (no readers)
                pass

    def rec_obj(self, sub_name: str):
        if sub_name in self.sub_pipe_paths:
            try:
                # Open the pipe in non-blocking mode
                fd = os.open(
                    self.sub_pipe_paths[sub_name], os.O_RDONLY | os.O_NONBLOCK)
                with os.fdopen(fd, 'rb') as pipe:
                    return pickle.load(pipe)
            except EOFError:
                return None
            except OSError as e:
                raise
        else:
            raise ValueError(
                f"No subscriber with name '{sub_name}' initialized.")

    def rec_obj_block(self, sub_name: str):
        if sub_name in self.sub_pipe_paths:
            with open(self.sub_pipe_paths[sub_name], "rb") as pipe:
                return pickle.load(pipe)
        else:
            raise ValueError(
                f"No subscriber with name '{sub_name}' initialized.")

    def close(self):
        # Closing operations are not necessary for named pipes like they are for sockets
        pass

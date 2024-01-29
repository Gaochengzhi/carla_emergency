import zmq
import multiprocessing
import time


def client1():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(f"tcp://*:{5666}")

    while True:
        try:
            message = socket.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 1 Received: {message}")
            pub_socket.send_pyobj({"start 111"})
        except zmq.Again:
            pass
        time.sleep(0.2)


def client2():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket2 = context.socket(zmq.SUB)
    socket2.connect("tcp://localhost:5666")
    socket2.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        try:
            message = socket.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 2 Received: {message}")
            message2 = socket2.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 2 Received from client 1: {message2}")
        except zmq.Again:
            pass
        time.sleep(0.5)


def client3():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket2 = context.socket(zmq.SUB)
    socket2.connect("tcp://localhost:5666")
    socket2.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        try:
            message = socket.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 3 Received: {message}")
            message2 = socket2.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 3 Received from client 1: {message2}")
        except zmq.Again:
            pass
        time.sleep(0.5)


if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
    socket2 = context.socket(zmq.SUB)
    socket2.connect("tcp://localhost:5666")
    p1 = multiprocessing.Process(target=client1)
    p2 = multiprocessing.Process(target=client2)
    p3 = multiprocessing.Process(target=client3)
    p1.start()
    p2.start()
    p3.start()
    # time.sleep(1)
    while True:
        socket.send_pyobj({"start"})
        pass
        time.sleep(0.1)
        # message = socket.recv_pyobj()
        # print(f"Received: {message}")

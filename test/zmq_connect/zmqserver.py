import zmq
import multiprocessing
import time


def client1():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        try:
            message = socket.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 1 Received: {message}")
        except zmq.Again:
            pass
        time.sleep(1)


def client2():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        try:
            message = socket.recv_pyobj(flags=zmq.NOBLOCK)
            print(f"Client 2 Received: {message}")
        except zmq.Again:
            pass
        time.sleep(0.5)


if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
    p1 = multiprocessing.Process(target=client1)
    p2 = multiprocessing.Process(target=client2)
    p1.start()
    p2.start()
    # time.sleep(1)
    while True:
        socket.send_pyobj({"start"})
        pass
        time.sleep(5)
        # message = socket.recv_pyobj()
        # print(f"Received: {message}")

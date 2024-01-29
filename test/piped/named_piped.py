import multiprocessing
import time
import os
import pickle

pipe1_path = "/tmp/pipe1"
pipe2_path = "/tmp/pipe2"

# Create named pipes if they don't already exist
if not os.path.exists(pipe1_path):
    os.mkfifo(pipe1_path)
if not os.path.exists(pipe2_path):
    os.mkfifo(pipe2_path)


def client1():
    while True:
        with open(pipe1_path, "rb") as pipe1:
            try:
                message = pickle.load(pipe1)
                print(f"Client 1 Received: {message}")
                with open(pipe2_path, "wb") as pipe2:
                    pickle.dump({"start 111"}, pipe2)
            except EOFError:
                pass
        time.sleep(0.2)


def client2():
    while True:
        with open(pipe1_path, "rb") as pipe1:
            try:
                message = pickle.load(pipe1)
                print(f"Client 2 Received: {message}")
            except EOFError:
                pass

        with open(pipe2_path, "rb") as pipe2:
            try:
                message2 = pickle.load(pipe2)
                print(f"Client 2 Received from client 1: {message2}")
            except EOFError:
                pass

        time.sleep(0.5)


def client3():
    while True:
        with open(pipe1_path, "rb") as pipe1:
            try:
                message = pickle.load(pipe1)
                print(f"Client 3 Received: {message}")
            except EOFError:
                pass

        with open(pipe2_path, "rb") as pipe2:
            try:
                message2 = pickle.load(pipe2)
                print(f"Client 3 Received from client 1: {message2}")
            except EOFError:
                pass

        time.sleep(0.5)


if __name__ == "__main__":
    p1 = multiprocessing.Process(target=client1)
    p2 = multiprocessing.Process(target=client2)
    p3 = multiprocessing.Process(target=client3)
    p1.start()
    p2.start()
    p3.start()

    while True:
        with open(pipe1_path, "wb") as pipe1:
            pickle.dump({"start"}, pipe1)
        time.sleep(0.1)

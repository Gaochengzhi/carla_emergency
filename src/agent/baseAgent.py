import multiprocessing
import threading
import time
from data.commuicate_manager import CommuniAgent
import logging


class BaseAgent(multiprocessing.Process):
    def __init__(self, agent_name, config, agent_port):
        super(BaseAgent, self).__init__()
        self.name = agent_name
        self.config = config
        self.communi_agent = self.init_communi_agent(self.name, agent_port)
        self.start_listener_thread()

    def start_listener_thread(self):
        self.stop_listener = threading.Event()
        self.listener_thread = threading.Thread(
            target=self.listen_for_main_message)
        self.listener_thread.start()

    def listen_for_main_message(self):
        while not self.stop_listener.is_set():
            main_msg = self.communi_agent.rec_obj("main")
            if main_msg == "end":
                logging.debug(f"{self.name} received {main_msg} message")
                self.close_agent()
                break
            if main_msg == "on":
                pass
                # logging.debug(f"{self.name} received main message: {main_msg}")

    def close_agent(self):
        self.communi_agent.close()
        self.stop_listener.set()
        self.listener_thread.join()
        self.listener_thread._stop()
        exit(0)

    def init_communi_agent(self, agent_name, agent_port):
        communi_agent = CommuniAgent(agent_name)
        communi_agent.init_publisher(
            agent_port)
        communi_agent.send_obj(f"{agent_name} started")
        communi_agent.init_subscriber("main",
                                      self.config["PortParameters"]["main_port"])
        return communi_agent
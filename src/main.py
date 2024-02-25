from tools.world_manager import WorldManager
from tools.config_manager import config as cfg
from view.debug_manager import DebugManager, set_bird_view
from tools.input_manager import recieve_args
from agent.traffic_agent import TrafficFlowManager
from data.commuicate_manager import CommuniAgent
from data.recorder_manager import DataRecorder

# from view.pygame_manager import PyGameAgent
from util import destroy_all_actors, time_const, log_time_cost
import time
import carla
import sys
import logging
import os
from tools.loader import load_agents


def main():
    args = recieve_args()
    config = cfg.merge(args)
    world_manager = WorldManager(config)
    world = world_manager.get_world()
    client = world_manager.get_client()
    TM = world_manager.get_traffic_manager()
    destroy_all_actors(world)
    main_com = MainCommuicator(config)
    main_com.send_obj("start")
    load_agents(config)
    TrafficFlowManager().start()
    @time_const(fps=24)
    def run_step(world):
        main_com.send_obj("on")
        world.tick()
    try:
        while True:
            run_step(world)
    finally:
        main_com.send_obj("end")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        TM.set_synchronous_mode(False)
        destroy_all_actors(world)
        # client.reload_world()
        time.sleep(1)
        main_com.close()
        logging.info("Simulation ended\n")


def MainCommuicator(config):
    world_control_center = CommuniAgent("World")
    world_control_center.init_publisher(config["main_port"])

    # world_control_center.init_subscriber(agent["name"],
    #  agent["port"])
    return world_control_center


if __name__ == "__main__":
    main()

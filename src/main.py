from world_manager import WorldManager
from config_manager import config as cfg
from view.debug_manager import DebugManager, set_bird_view
from input_manager import recieve_args
from agent.traffic_agent import TrafficFlowManager
from agent.ego_vehicle_agent import EgoVehicleAgent
from data.commuicate_manager import CommuniAgent
from view.pygame_manager import PyGameAgent
from util import destroy_all_actors, time_const
import time
import carla
import logging
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'


def main():
    args = recieve_args()
    config = cfg.merge(args)
    carla_world = WorldManager(config)
    map, world, client, urban_waypoints, traffic_agent = get_world_instence(
        carla_world)

    main_com = MainCommuicator(config)
    main_com.send_obj("start")

    set_bird_view(world, urban_waypoints[1].transform.location,
                  config)
    start_point = urban_waypoints[1]
    end_point = urban_waypoints[200]

    EgoVehicleAgent(
        start_point, end_point, urban_waypoints, config).start()

    PyGameAgent(urban_waypoints, config).start()

    t = TrafficFlowManager(world, traffic_agent, urban_waypoints, config)
    t.start()

    @time_const(fps=25)
    def run_step(world):
        main_com.send_obj("on")
        info = main_com.rec_obj("emergency_vehicle")
        if info:
            logging.debug(f"main received info: {info}")
        world.tick()

    try:
        while True:
            run_step(world)
    finally:
        main_com.send_obj("end")
        time.sleep(1)
        destroy_all_actors(world)
        main_com.close()
        logging.info("Simulation ended\n")
        time.sleep(1)


def MainCommuicator(config):
    world_control_center = CommuniAgent("World")
    world_control_center.init_publisher(config["PortParameters"]["main_port"])
    world_control_center.init_subscriber("emergency_vehicle",
                                         6001)
    return world_control_center


def get_world_instence(scenario_loader):
    map = scenario_loader.get_map()
    world = scenario_loader.get_world()
    client = scenario_loader.get_client()
    urban_waypoints = scenario_loader.get_filtered_waypoints()
    traffic_agent = scenario_loader.get_trafficmanager()
    world.tick()
    return map, world, client, urban_waypoints, traffic_agent


if __name__ == "__main__":
    main()

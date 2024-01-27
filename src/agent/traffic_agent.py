import random

import logging
import carla
from agent.ego_vehicle_agent import EgoVehicleAgent
from view.debug_manager import DebugManager as debug
from util import spawn_vehicle, connect_to_server, time_const
from agent.baseAgent import BaseAgent
import time


class TrafficFlowManager(BaseAgent):
    def __init__(
        self,
        world,
        traffic_agent,
        spawn_waypoints,
        config,
    ) -> None:
        self.init_traffic_flow(world, traffic_agent, spawn_waypoints, config)
        BaseAgent.__init__(self, "TrafficFlow", config,
                           config["PortParameters"]["traffic_agent_port"])

    def init_traffic_flow(self, world, traffic_agent, spawn_waypoints, config):
        self.config = config
        num_vehicles = self.config["TrafficParameters"]["num_vehicles"]
        v_types = self.config["TrafficParameters"]["bg_vehicle_type"]
        self.spawn_waypoints = spawn_waypoints
        self.create_bg_vehicles(
            world, num_vehicles, v_types, self.spawn_waypoints, traffic_agent)

    @time_const(fps=1)
    def run_step(self):
        print("traffic flow running")
        pass


    def run(self):
        client, world = connect_to_server(self.config)
        self.communi_agent.init_subscriber("EgoVehicle", self.config["PortParameters"]["ego_port"])

        while True:
            info = self.communi_agent.rec_obj("EgoVehicle")
            logging.debug(f"traffic flow received info: {info}")
            self.run_step()
            pass




    def choose_a_point(self, waypoint_list):
        choosen_waypoint = random.choice(waypoint_list)
        waypoint_list.remove(choosen_waypoint)
        return choosen_waypoint

    def create_bg_vehicles(self, world, num_vehicles, vehicle_types, spawn_waypoints, traffic_agent):
        bg_vehicles = []
        for _ in range(num_vehicles):
            try:
                spawn_point = self.choose_a_point(spawn_waypoints)
                v_type = random.choice(vehicle_types)
                vehicle = spawn_vehicle(world,
                                        v_type, spawn_point)
                while vehicle is None:
                    logging.warn(
                        f"spawn_actor{v_type} failed, trying another start point...")
                    v_type = random.choice(vehicle_types)
                    spawn_point = self.choose_a_point(spawn_waypoints)
                    vehicle = spawn_vehicle(world, v_type, spawn_point)
                if vehicle is not None:
                    vehicle.set_autopilot(True, traffic_agent.get_port())
                    # world.wait_for_tick()
                bg_vehicles.append(vehicle)
            except Exception as e:
                logging.error(f"create traffic flow error:{e}")
        logging.debug(
            f"spawned {len(bg_vehicles)} vehicles")
        return bg_vehicles

    def get_bg_vehicles(self):
        return self.bg_vehicles

    def get_ego_vehicle(self):
        return self.ego_vehicle.vehicle

    def get_sensor_manager(self):
        return self.ego_vehicle.sensor_manager

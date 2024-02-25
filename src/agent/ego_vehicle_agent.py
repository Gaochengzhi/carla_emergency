from agent.baseAgent import BaseAgent
from data.commuicate_manager import CommuniAgent
from util import connect_to_server, spawn_vehicle, time_const, is_within_distance, compute_distance, log_time_cost, txt_to_points
from view.debug_manager import draw_waypoints_arraw, draw_transforms, set_bird_view

from preception.sensorManager import SensorManager
from navigation.global_route_planner import GlobalRoutePlanner
from navigation.controller import VehiclePIDController
from plan.frenetPlanner import FrenetPlanner
import carla
import logging
import time
import math
import random


class EgoVehicleAgent(BaseAgent):
    def __init__(self, config):
        self.config = config
        BaseAgent.__init__(
            self, self.config["name"], self.config["port"])
        self.count = 0

    def run(self):
        client, world = connect_to_server(8, 2000)
        self.start_agent()
        self.set_communi_agent()
        start_point, end_point = self.get_navi_pos(world)
        set_bird_view(world, start_point.location, 100)
        self.vehicle = self.create_vehicle(world, start_point,
                                           self.config["type"])
        self.global_route_planner = GlobalRoutePlanner(
            world.get_map(), sampling_resolution=6)
        self.global_router_waypoints =  [x[0] for x in self.global_route_planner.trace_route(
            start_point.location, end_point.location)]
        self.local_planner = FrenetPlanner(
            world, self.global_router_waypoints, self.vehicle,self.config, VehiclePIDController)
        control = carla.VehicleControl()
        try:
            while True:
                if len(self.global_router_waypoints) < 1:
                    logging.info("vehicle reach destination")
                    self.close_agent()
                    return
                self.run_step(control)

                # DEBUG

        except KeyboardInterrupt:
            self.close_agent()
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
            exit()

    def get_navi_pos(self, world):
        self.waypoints = world.get_map().get_spawn_points()
        start_point = self.waypoints[self.config["start_point"]]
        end_point = self.waypoints[self.config["end_point"]]
        return start_point, end_point

    def run_step(self, control):
        obs=self.communi_agent.rec_obj("router")
        self.local_planner.run_step(obs, control)
        time.sleep(0.03)



    def set_communi_agent(self):
        self.communi_agent.init_subscriber("router",
                                           self.config["traffic_agent_port"])
        pass

    def create_vehicle(self, world, start_point, ego_vehicle_type):
        try:
            spawn_actor = spawn_vehicle(
                world, ego_vehicle_type, start_point, hero=True)
            while spawn_actor is None:
                logging.info(
                    f"spawn_actor{ego_vehicle_type} failed, trying another start point...")
                start_point = random.choice(self.waypoints)
                spawn_actor = spawn_vehicle(
                    world, ego_vehicle_type, start_point, hero=True)
            # world.wait_for_tick()
            return spawn_actor
        except Exception as e:
            logging.error(f"create ego vehicle error:{e}")
            raise

    def get_trajection(self):
        return self.trajection

    def get_vehicle(self):
        return self.vehicle

    def get_count(self):
        return self.count

    def send_destination(self, start, end):
        # get_info = self.communi_agent.rec_obj("router")
        # while get_info != "get":
        self.communi_agent.send_obj([start.id, end.id])
        # logging.debug(f"send destination: {start.id}, {end.id}")
        # get_info = self.communi_agent.rec_obj("router")
        # time.sleep(0.1)

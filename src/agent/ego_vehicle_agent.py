from agent.baseAgent import BaseAgent
from data.commuicate_manager import CommuniAgent
from util import connect_to_server, spawn_vehicle, time_const, is_within_distance, compute_distance, log_time_cost, txt_to_points
from view.debug_manager import draw_waypoints_arraw, draw_transforms, set_bird_view
from navigation.global_route_planner import GlobalRoutePlanner
from control.local_planner import LocalPlanner
import carla
import logging
import time
import math
import random


class EgoVehicleAgent(BaseAgent):
    def __init__(self,config):
        self.config = config
        BaseAgent.__init__(
            self, self.config["name"], self.config["port"] )
        self.count = 0

    def run(self):
        client, world = connect_to_server(8, 2000)
        self.start_agent()
        self.set_communi_agent()
        self.waypoints = world.get_map().get_spawn_points()

        start_point = self.waypoints[self.config["start_point"]]
        end_point = self.waypoints[self.config["end_point"]]

        set_bird_view(world, start_point.location, 50)
        self.vehicle = self.create_vehicle(world, start_point,
                                           self.config["type"])
        self.global_route_planner = GlobalRoutePlanner(
            world.get_map(), 5)
        logging.info("global route planner init success")

        self.global_router_waypoints = self.global_route_planner.trace_route(
            start_point.location, end_point.location)

        # self.local_planner = LocalPlanner(
        #     world, self.global_router_waypoints, self.vehicle, self.config)

        control = carla.VehicleControl()
        control.throttle = 0.5
        self.vehicle.set_autopilot(True)
        while True:

            if len(self.global_router_waypoints) == 0:
                logging.info("vehicle reach destination")
                self.close_agent()
                return

            # DEBUG
            gw = [x[0] for x in self.global_router_waypoints]
            draw_waypoints_arraw(world, gw, z=2, life_time=1)
            # self.vehicle.apply_control(control)

            # draw_transforms(world, self.local_planner.get_trajection(
            # ), color=carla.Color(0, 128, 0), size=0.03, life_time=1)
            time.sleep(1)

            # control = self.local_planner.run_step(speed=23)
            # self.run_step(control)

    # @time_const(fps=25)

    def run_step(self, control):
        # self.send_destination(self.start_point, self.end_point)
        # obs_vehicles = self.communi_agent.rec_obj("router")
        obs_vehicles = []
        if obs_vehicles:
            min_distance = 2
            for obs_vehicle in obs_vehicles:
                if obs_vehicle:
                    obs_distance = compute_distance(
                        self.vehicle.get_location(), self.local_planner.get_trajection()[19].location)
                    if obs_distance < min_distance:
                        logging.debug(f"obs_distance: {obs_distance}")
                        min_distance = obs_distance
                        control.brake = 0.1*(2-obs_distance)
                        # logging.debug(f"ego vehicle brake from obs: {control.brake}")
                        control.throttle = 0
            control.brake = 0.1*(3-min_distance)
            # logging.debug(f"ego vehicle brake: {control.brake}")
            control.throttle = 0
        self.vehicle.apply_control(control)

    def set_communi_agent(self):
        self.communi_agent.init_subscriber("router",
                                           self.config["traffic_agent_port"])
        pass

    def create_vehicle(self, world, start_point, ego_vehicle_type):
        try:
            spawn_actor = spawn_vehicle(
                world, ego_vehicle_type, start_point, hero=True)
            world.wait_for_tick()
            while spawn_actor is None:
                logging.info(
                    f"spawn_actor{ego_vehicle_type} failed, trying another start point...")
                start_point = random.choice(self.waypoints)
                spawn_actor = spawn_vehicle(
                    world, ego_vehicle_type, start_point, hero=True)
            world.wait_for_tick()
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

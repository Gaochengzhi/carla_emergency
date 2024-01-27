from view.debug_manager import set_bird_view
from agent.baseAgent import BaseAgent
from data.commuicate_manager import CommuniAgent
from util import connect_to_server, spawn_vehicle, time_const
import time
import carla
import logging
import random


class EgoVehicleAgent(BaseAgent):
    def __init__(self,  start_point, end_point, waypoints, config):
        BaseAgent.__init__(self, "EgoVehicle", config,
                           config["PortParameters"]["ego_port"])
        self.start_point = start_point
        self.end_point = end_point
        self.waypoints = waypoints
        self.count = 0

    def run(self):
        client, world = connect_to_server(self.config)
        self.set_communi_agent()
        self.vehicle = self.create_vehicle(world, self.start_point,
                                           self.config["EmergencyVehicleParameters"]["ego_vehicle_type"])
        
        control = carla.VehicleControl()
        control.throttle = 3.0
        control.steer = 0.0
        while True:
            self.send_destination(self.start_point, self.end_point)
            self.run_step(control)

    @time_const(fps=0.1)
    def send_destination(self,start,end):
        self.communi_agent.send_obj({
            "title": "router",
            "content": [start.id,end.id]
        })
        logging.debug(f"send destination {start.id} {end.id}")

    def run_step(self, control):
        vehicle_location = self.vehicle.get_location()
        # self.vehicle.apply_control(control)

    def set_communi_agent(self):
        pass
        # self.communi_agent.init_subscriber("router",
        #                                    self.config["PortParameters"]["traffic_agent_port"])

    def create_vehicle(self, world, start_point, ego_vehicle_type):
        try:
            spawn_actor = spawn_vehicle(
                world, ego_vehicle_type, start_point, hero=True)
            world.wait_for_tick()
            while spawn_actor is None:
                logging.info(
                    f"spawn_actor{ego_vehicle_type} failed, trying another start point...")
                spawn_actor = spawn_vehicle(
                    world, ego_vehicle_type, random.choice(self.waypoints), hero=True)
            world.wait_for_tick()
            return spawn_actor
        except Exception as e:
            logging.error(f"create ego vehicle error:{e}")
            raise

    def get_global_plan(self):
        return self.global_plan

    def get_local_plan(self):
        return self.local_plan

    def get_trajection(self):
        return self.trajection

    def get_vehicle(self):
        return self.vehicle

    def get_count(self):
        return self.count

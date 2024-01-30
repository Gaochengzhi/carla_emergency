from view.debug_manager import set_bird_view
from agent.baseAgent import BaseAgent
from data.commuicate_manager import CommuniAgent
from util import connect_to_server, spawn_vehicle, time_const, is_within_distance
from view.debug_manager import draw_waypoints_arraw, draw_transforms
from navigation.global_route_planner import GlobalRoutePlanner
from control.local_planner import LocalPlanner
import carla
import logging
import random


class EgoVehicleAgent(BaseAgent):
    def __init__(self,  start_point, end_point, waypoints, config):
        ego_port = config["PortParameters"]["ego_port"]
        self.ego_type = config["EmergencyVehicleParameters"]["ego_vehicle_type"]
        self.router_resolution = config["EmergencyVehicleParameters"]["global_route_resolution"]

        BaseAgent.__init__(self, "EgoVehicle", config,
                           ego_port)
        self.start_point = start_point
        self.config = config
        self.end_point = end_point
        self.waypoints = waypoints
        self.global_router_waypoints = None
        self.count = 0

    def run(self):
        client, world = connect_to_server(self.config)
        self.set_communi_agent()
        self.vehicle = self.create_vehicle(world, self.start_point,
                                           self.ego_type)
        global_route_planner = GlobalRoutePlanner(
            world.get_map(), self.router_resolution)
        self.global_router_waypoints = global_route_planner.trace_route(
            self.start_point.transform.location, self.end_point.transform.location)


        local_planner = LocalPlanner(world, self.global_router_waypoints,self.vehicle,self.config)
        control = carla.VehicleControl()
        control.throttle = 3.0
        control.steer = 0.0
        while True:
            if len(self.global_router_waypoints)==0:
                logging.info("vehicle reach destination")
                self.close_agent()
            
            gw = [x[0] for x in self.global_router_waypoints]
            draw_waypoints_arraw(world, gw, z=2, life_time=1)
            draw_transforms(world, local_planner.get_trajection(), color=carla.Color(0,128,0),size=0.03,life_time=1)
            
            control = local_planner.run_step(speed=33)
            if control is not None:
                self.run_step(control)

    @time_const(fps=3)
    def send_destination(self, start, end):
        self.communi_agent.send_obj('sdajkdhaks')
        logging.debug(f"send destination {start.id} {end.id}")

    @time_const(fps=25)
    def run_step(self, control):
        self.vehicle.apply_control(control)

    def set_communi_agent(self):
        # self.communi_agent.init_subscriber("router",
        #                                    self.config["PortParameters"]["traffic_agent_port"])
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

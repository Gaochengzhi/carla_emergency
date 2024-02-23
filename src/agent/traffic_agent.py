import random
import math
import os
import logging
import carla
from agent.ego_vehicle_agent import EgoVehicleAgent
from prediction.prediction_phy import predict
from view.debug_manager import draw_future_locations
from util import connect_to_server, time_const, log_time_cost,batch_process_vehicles, get_speed
from agent.baseAgent import BaseAgent
import time
from tools.config_manager import config as cfg


class TrafficFlowManager(BaseAgent):
    def __init__(
        self,
    ) -> None:
        self.config = cfg.config
        self.fps = 2
        BaseAgent.__init__(self, "TrafficFlow",
                           self.config["traffic_agent_port"])

    

    @log_time_cost
    @time_const(fps=10)
    def run_step(self, world):
        preception_res = batch_process_vehicles(world, predict, self.fps)
        draw_future_locations(world, preception_res, life_time=1)
        # logging.debug(f"traffic flow received info: {preception_res}")
        self.communi_agent.send_obj(preception_res)

        # control = carla.VehicleControl()
        # threshold_long_distance = 30
        # ego_lane_index = self.get_lane_id(ego_vehicle.get_location())
        # current_location = ego_vehicle.get_location()
        # res = batch_process_vehicles(world, ego_vehicle, 10,
        #                              [-3, 3], self.obstacle_change_lane, self.traffic_manager, control, threshold_long_distance, ego_lane_index, current_location)
        # if res:
        #     self.communi_agent.send_obj(res)
        pass

    def run(self):
        client, world = connect_to_server(
            self.config["carla_timeout"], self.config["carla_port"])
        self.map = world.get_map()
        self.start_agent()
        time.sleep(1)
        while True:
            self.run_step(world)
            pass

        # self.traffic_manager = client.get_trafficmanager()
        # self.communi_agent.init_subscriber("EgoVehicle",
        #                                      self.config["ego_port"])

        # time.sleep(3)
        # ego_vehicle = get_ego_vehicle(world)
        # while True:
        #     # info = self.communi_agent.rec_obj("EgoVehicle")
        #     # logging.debug(f"traffic flow received info: {info}")
        #     self.run_step(world,ego_vehicle)
        #     pass
    def obstacle_change_lane(self, world, obstacle, ego_v, tm, control, threshold_long_distance, ego_lane_index, current_location):

        # control = carla.VehicleControl()
        # control.brake = 2
        # obstacle.apply_control(control)
        return
        obstacle_location = obstacle.get_location()
        obstacle_yaw = obstacle.get_transform().rotation.yaw
        obs_speed = get_speed(obstacle)
        distance_obs = math.sqrt(
            (obstacle_location.x - current_location.x) ** 2
            + (obstacle_location.y - current_location.y) ** 2
        )
        obs_lane_index = self.get_lane_id(obstacle_location)
        lane_shit = -1.1 if abs(obs_lane_index) == 1 else 1.1
        if distance_obs < threshold_long_distance + 10 and distance_obs > 10:
            if obs_lane_index == ego_lane_index:
                if abs(obs_lane_index) == 1:
                    tm.force_lane_change(
                        obstacle, False
                    )  # -1 means left and 1 means right
                else:
                    tm.force_lane_change(
                        obstacle, True
                    )  # -1 means left and 1 means right
            # -1 means left and 1 means right
            self.block = False
            if (
                distance_obs < threshold_long_distance + 10
                # and obs_speed < 2
                # and distance_obs > 5
            ):
                control.steer = lane_shit * 0.55 * (1.9 - obs_speed)
                # control.throttle = 2
                control.steer = 2
                # control.brake = 2
                obstacle.apply_control(control)
            elif distance_obs < threshold_long_distance - 8:
                tm.vehicle_lane_offset(obstacle, lane_shit)

            if distance_obs < threshold_long_distance + 5 and (
                obs_lane_index == ego_lane_index
            ):
                self.block = True
                self.step = 90
                self.lane_shift = 1.7 if abs(ego_lane_index) == 1 else -1.7
        pass

    def get_sensor_manager(self):
        return self.ego_vehicle.sensor_manager


def main():
    TrafficFlowManager().run()


if __name__ == "__main__":
    main()

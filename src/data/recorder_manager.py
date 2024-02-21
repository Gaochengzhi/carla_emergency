import random
import math
import os
import logging
import carla
from agent.ego_vehicle_agent import EgoVehicleAgent
from view.debug_manager import DebugManager as debug
from util import spawn_vehicle, connect_to_server, time_const, batch_process_vehicles, get_ego_vehicle, get_speed
from agent.baseAgent import BaseAgent
import time
import csv


class DataRecorder(BaseAgent):
    def __init__(
        self,
        world,
        traffic_agent,
        spawn_waypoints,
        config,
    ) -> None:
        BaseAgent.__init__(self, "DataRecorder", config,
                           config["data_port"])

    def run(self):
        client, world = connect_to_server(
            self.config["carla_timeout"], self.config["carla_port"])
        self.start_agent()
        self.ego_vehicle = get_ego_vehicle(world)
        writer = self.init_data_file(
            self.config["DataParameters"]["data_folder_path"])

        writer.writerow([
            'time',
            'vehicle_id',
            'location_x',
            'location_y',
            'location_z',
            'velocity_x',
            'velocity_y',
            'velocity_z',
            'acceleration_x',
            'acceleration_y',
            'acceleration_z',
            'angular_velocity_x',
            'angular_velocity_y',
            'angular_velocity_z',
            'rotation_roll',
            'rotation_pitch',
            'rotation_yaw',
            'control_brake',
            'control_throttle',
            'control_steer'
        ])
        while True:
            self.run_step(world, writer)

    @time_const(fps=20)
    def run_step(self, world, writer):
        current_time = time.time()
        v_list = batch_process_vehicles(world, self.ego_vehicle, 200,
                                        [-180, 180], self.get_vehicle_info, writer, time_now=current_time)

    def get_vehicle_info(self, world, vehicle, ego_vehicle, writer, time_now):
        # Get vehicle transform (location and rotation)
        transform = vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation

        # Get vehicle control inputs
        control = vehicle.get_control()

        # Get vehicle ID
        v_id = vehicle.id

        # Calculate acceleration
        # Note: This is a basic approach. For a more accurate calculation, you would need additional data.
        velocity = vehicle.get_velocity()
        acceleration = vehicle.get_acceleration()
        angular_velocity = vehicle.get_angular_velocity()

        # Write data to CSV
        writer.writerow([
            time_now,
            v_id,
            location.x,
            location.y,
            location.z,
            velocity.x,
            velocity.y,
            velocity.z,
            acceleration.x,
            acceleration.y,
            acceleration.z,
            angular_velocity.x,
            angular_velocity.y,
            angular_velocity.z,
            rotation.roll,
            rotation.yaw,
            rotation.pitch,
            control.brake,
            control.throttle,
            control.steer,
        ])

        pass

    def init_data_file(self, folder_path):
        current_time = time.time()
        logging.info(f"creat_data_file: {current_time}")
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        self.fp = open(folder_path+"/data.csv", "w")
        writer = csv.writer(self.fp)
        return writer
        # write csv file header

    def close(self) -> None:
        self.fp.fflush()
        self.fp.close()
        time.sleep(2)
        return super().close()

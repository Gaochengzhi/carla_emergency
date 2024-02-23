import random
import math
import os
import logging
import carla
from agent.ego_vehicle_agent import EgoVehicleAgent
from view.debug_manager import DebugManager as debug
from util import spawn_vehicle, connect_to_server, time_const, batch_process_vehicles, get_ego_vehicle, get_speed, log_time_cost,get_vehicle_info
from agent.baseAgent import BaseAgent
import time
import csv


class DataRecorder(BaseAgent):
    def __init__(
        self,
        config,
    ) -> None:
        self.config = config
        BaseAgent.__init__(self, "DataRecorder", 
                           config["data_port"])

    def run(self):
        client, world = connect_to_server(
            self.config["carla_timeout"], self.config["carla_port"])
        self.start_agent()
        # self.ego_vehicle = get_ego_vehicle(world)
        writer = self.init_data_file(
            self.config["data_folder_path"])

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
            'control_steer',
            'torque_curveX',
            'torque_curveY',
            'moi',
            'mass',
            'drag_coefficient',
            'center_of_mass',
            'w1tyre_friction',
            'w1damping_rate',
            'w2tyre_friction',
            'w2damping_rate',
            'w3tyre_friction',
            'w3damping_rate',
            'w4tyre_friction',
            'w4damping_rate' 
        ])
        while True:
            self.run_step(world, writer)

    # @log_time_cost
    @time_const(fps=2)
    def run_step(self, world, writer):
        current_time = time.time()
        v_list = batch_process_vehicles(world, self.write_vehicle_info, writer, time_now=current_time)

    def write_vehicle_info(self, world, vehicle, writer, time_now):
        # Assuming 'vehicle' is the target vehicle from which we are extracting data
        
        # Basic info
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        acceleration = vehicle.get_acceleration()
        angular_velocity = vehicle.get_angular_velocity()
        transform = vehicle.get_transform()
        control = vehicle.get_control()
        
        # Physics control info
        physics_control = vehicle.get_physics_control()
        wheels = physics_control.wheels
        
        # Extracting data from physics_control
        torque_curve = [(point.x, point.y) for point in physics_control.torque_curve]
        center_of_mass = physics_control.center_of_mass
        
        # Writing data to CSV
        writer.writerow([
            time_now,
            vehicle.id,
            location.x, location.y, location.z,
            velocity.x, velocity.y, velocity.z,
            acceleration.x, acceleration.y, acceleration.z,
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw,
            control.brake, control.throttle, control.steer,
            # Assuming you're interested in the first point of the torque curve for this example
            torque_curve[0][0] if torque_curve else 0, torque_curve[0][1] if torque_curve else 0,
            physics_control.moi,
            physics_control.mass,
            physics_control.drag_coefficient,
            f"{center_of_mass.x}, {center_of_mass.y}, {center_of_mass.z}",
            # Wheels data - assuming 4 wheels and extracting tyre friction and damping rate for each
            *(wheels[i].tire_friction for i in range(4)),
            *(wheels[i].damping_rate for i in range(4))
        ])

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

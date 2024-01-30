from util import is_within_distance, compute_distance
import logging
import carla
import numpy as np
from scipy.interpolate import splprep, splev
from navigation.controller import VehiclePIDController


class LocalPlanner():
    def __init__(self,world,global_waypoints, ego_vehicle,config) -> None:
        self.global_waypoints = global_waypoints
        self.ego_vehicle = ego_vehicle
        self.local_planner_buffer = []
        self.local_sample_resolution = config["EmergencyVehicleParameters"]["local_planner_resolution"]
        self.global_sample_resolution = config["EmergencyVehicleParameters"]["global_route_resolution"]
        self._dt = 1.0 / config["PygameParameters"]["screen_fps"]
        self._args_lateral_dict = {'K_P': 1.95,
                                   'K_I': 0.05, 'K_D': 0.2, 'dt': self._dt}
        self._args_longitudinal_dict = {
            'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': self._dt}
        self.controller = VehiclePIDController(self.ego_vehicle, self._args_lateral_dict, self._args_longitudinal_dict)

    def run_step(self,speed):
        ego_location = self.ego_vehicle.get_transform()
        wp = [x[0] for x in self.global_waypoints]
        if len(self.local_planner_buffer) < 20:
            self.local_planner_buffer = self.generate_smooth_trajectory(self.ego_vehicle, wp)

        if is_within_distance(ego_location, self.global_waypoints[0][0].transform, self.global_sample_resolution+3):
            self.global_waypoints.pop(0)
            pass

        if is_within_distance(ego_location, self.local_planner_buffer[0], self.local_sample_resolution):
            self.local_planner_buffer.pop(0)
            pass
        
        res = self.controller.run_step(speed, self.local_planner_buffer[0])
        return res



        pass

    def generate_smooth_trajectory(self, ego, waypoints):
            waypoints = waypoints[:min(len(waypoints), 9)]
            # Ensure we have at least 2 waypoints
            if len(waypoints) < 2:
                return []

            # Filter waypoints that are less than 0.5 meters apart
            filtered_waypoints = [waypoints[0]]
            for waypoint in waypoints[1:]:
                last_waypoint = filtered_waypoints[-1]
                distance = compute_distance(last_waypoint.transform.location, waypoint.transform.location)
                if distance >= 0.5:
                    filtered_waypoints.append(waypoint)

            # Extract x, y, z and yaw from filtered waypoints
            xs, ys, zs, yaws,rolls,pitchs = [], [], [], [],[],[]
            for waypoint in filtered_waypoints:
                location = waypoint.transform.location
                rotation = waypoint.transform.rotation
                xs.append(location.x)
                ys.append(location.y)
                zs.append(location.z)
                yaws.append(rotation.yaw)
                rolls.append(rotation.roll)
                pitchs.append(rotation.pitch)

            # Calculate total distance
            total_distance = sum([compute_distance(filtered_waypoints[i].transform.location, filtered_waypoints[i + 1].transform.location) 
                                for i in range(len(filtered_waypoints) - 1)])
            
            # Number of samples
            num_samples = int(total_distance / 2) + 1

            # Cubic spline for position
            tck, _ = splprep([xs, ys, zs], s=0)
            u_fine = np.linspace(0, 1, num_samples)

            # Interpolate positions
            x_fine, y_fine, z_fine = splev(u_fine, tck)

            # Cubic spline for yaw
            try:
                yaw_tck, _ = splprep([yaws], s=0)
                yaws_fine = splev(u_fine, yaw_tck)[0]
                roll_tck, _ = splprep([yaws], s=0)
                rolls_fine = splev(u_fine, roll_tck)[0]
                pitch_tck, _ = splprep([yaws], s=0)
                pitches_fine = splev(u_fine, pitch_tck)[0]
            except:
                yaws_fine = [yaws[0] for _ in range(num_samples)]
                rolls_fine = [rolls[0] for _ in range(num_samples)]
                pitches_fine = [pitchs[0] for _ in range(num_samples)]

            

            # Combine position and rotiation into carla transform objects
            soomth_trajectory = []
            for i in range(len(x_fine)):
                transform = carla.Transform()
                transform.location.x = x_fine[i]
                transform.location.y = y_fine[i]
                transform.location.z = z_fine[i]
                transform.rotation.yaw = yaws_fine[i]
                transform.rotation.roll = rolls_fine[i]
                transform.rotation.pitch = pitches_fine[i]
                soomth_trajectory.append(transform)

            return soomth_trajectory
    def get_trajection(self):
            return self.local_planner_buffer
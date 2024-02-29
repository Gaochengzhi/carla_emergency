
import time
import functools
import numpy as np
import math
import random
import sys
import carla
from util import compute_distance, is_within_distance_obs, log_time_cost, compute_3D21d, compute_distance2D, compute_magnitude_angle
from view.debug_manager import draw_waypoints, draw_list
from scipy.interpolate import splprep, splev
from plan.carFollowingModel import IDM
import logging


class FrenetPlanner():
    def __init__(self, world, map, g_waypoints, vehicle, config, controller):
        self.global_paths = g_waypoints
        self.world = world
        self.map = map
        self.config = config
        self.vehicle = vehicle
        self.self_id = vehicle.id
        self.obs_list = []
        self.waypoint_buffer = []
        self.local_traj = []
        self.left_side = 1.5
        self.right_side = 1.5
        self.ego_speed = 0
        self.acceleration = 0
        self.transform = None
        self.controller = controller(vehicle)
        self.car_following_model = IDM()
        self.target_speed = random.randint(85, 90)
        self.location = None
        self.use_car_following = False
        self.vehicle.show_debug_telemetry(True)
        self.ignore_traffic_light = config["ignore_traffic_light"]
        self.get_vehicle_info()
        self.check_update_waypoints()

    # @log_time_cost(name="ego_planner")
    def run_step(self, obs, control):
        # get_info
        try:
            self.get_vehicle_info()
            self.get_obstacle(
                obs, self.transform, self.self_id, self.ego_speed)
            # check finish
            if len(self.global_paths) < 1:
                self.vehicle.destroy()
                logging.info("finish the route!")
                exit()

            if compute_distance(self.waypoint_buffer[0].transform.location, self.vehicle.get_location()) < 7:
                self.waypoint_buffer.pop(0)
                self.global_paths.pop(0)
                self.update_waypoint_buffer(self.ego_speed)
                self.update_trajectories(-5)
                self.check_collison(self.obs_list, self.local_traj)
            # check collison
            if len(self.local_traj) < 1:
                return

            if compute_distance2D(self.local_traj[0], [self.location.x, self.location.y]) < 5:
                self.local_traj.pop(0)
                # self.check_collison(self.obs_list, self.local_traj)
            if len(self.local_traj) < 1:
                return
            # Frenet Optimal Trajectory plan
            # DEBUG
            # draw_waypoints(self.world, self.waypoint_buffer,
            #                z=1, life_time=0.1, size=0.3)
            draw_list(self.world, self.local_traj, size=0.2,
                      color=carla.Color(0, 250, 123), life_time=0.1)

            # CONTROLLER
            target_waypoint = carla.Transform(
                carla.Location(x=self.local_traj[0][0], y=self.local_traj[0][1], z=1))
            control = self.controller.run_step(
                self.target_speed, target_waypoint)
            self.vehicle.apply_control(control)
        except Exception as e:
            logging.error(f"plan run_step error: {e}")
            print(e.__traceback__.tb_frame.f_globals["__file__"])
            print(e.__traceback__.tb_lineno)
            pass

    def adjust_traj(self, ob_list):
        traj_adjust_opt = np.arange(-self.left_side, self.right_side+1, 1)
        for offset in traj_adjust_opt:
            self.update_trajectories(offset=offset)
            if not self.check_collison(ob_list, self.local_traj):
                return

    def check_update_waypoints(self):
        if len(self.waypoint_buffer) < 1:
            self.update_waypoint_buffer(self.ego_speed)
        if len(self.local_traj) < 1:
            self.update_waypoint_buffer(self.ego_speed)
            self.update_trajectories(-5)

    @log_time_cost(name="update_trajectories")
    def update_trajectories(self, offset=0):
        xy_list = [[waypoint.transform.location.x, waypoint.transform.location.y]
                   for waypoint in self.waypoint_buffer]
        # Remove duplicates
        xy_list = [xy_list[i] for i in range(
            len(xy_list)) if i == 0 or xy_list[i] != xy_list[i-1]]

        selfloc = [self.location.x, self.location.y]
        xy_list.insert(0, selfloc)
        xy_list = np.array(xy_list)
        lenxy = len(xy_list)

        # Ensure there are enough points for spline interpolation
        if lenxy > 3:
            tck, u = splprep(xy_list.T, s=2, k=3)
        elif lenxy < 4 and lenxy > 1:
            tck, u = splprep(xy_list.T, s=2, k=1)
        else:
            self.local_traj = xy_list.tolist()
            return

        u_new = np.linspace(u.min(), u.max(), lenxy * 4)
        x_fine, y_fine = splev(u_new, tck)

        # Calculate normal vectors for each point on the curve
        dx, dy = np.gradient(x_fine), np.gradient(y_fine)
        normals = np.vstack((-dy, dx))
        normals /= np.linalg.norm(normals, axis=0)  # Normalize

        # Decide on initial offset direction based on the vehicle's current position
        # This step assumes calculation of an initial direction or side, which is simplified here.
        # In practice, you may need to analyze the vehicle's current position relative to the path more deeply.

        # Apply the desired offset to each point along the path
        if offset != 0:
            x_offset = x_fine + normals[0] * offset
            y_offset = y_fine + normals[1] * offset
            interpolated_waypoints = np.column_stack(
                [x_offset, y_offset]).tolist()
        else:
            interpolated_waypoints = np.column_stack([x_fine, y_fine]).tolist()

        self.local_traj = interpolated_waypoints

    def check_collison(self, obs, array):
        ob_list = []
        for ob in obs:
            for point in array:
                if compute_distance2D((point[0], point[1]), (ob[0], ob[1])) < 2:
                    ob_list.append(ob)
        if ob_list:
            self.adjust_traj(ob_list)
            return True

    def update_waypoint_buffer(self, speed):
        num_push = min(int(speed*2/5)+1, len(self.global_paths))
        self.waypoint_buffer = self.global_paths[:num_push]

    def get_vehicle_info(self):
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        transform = self.vehicle.get_transform()
        acceleration = self.vehicle.get_acceleration()

        distance_to_left_side, distance_to_right_side = self.get_road_bound(
            location)
        c_speed = compute_3D21d(velocity)
        c_accel = compute_3D21d(acceleration)
        self.transform = transform
        self.location = location
        self.ego_speed = c_speed
        self.left_side = distance_to_left_side
        self.right_side = distance_to_right_side
        self.acceleration = c_accel

    # @log_time_cost(name="get_road_bound")
    def get_road_bound(self, location):
        waypoint_ego = self.map.get_waypoint(location, project_to_road=False)
        waypoint_road = self.map.get_waypoint(location)
        if waypoint_ego and waypoint_road:
            lane_width = waypoint_road.lane_width
            left_lane = waypoint_road.get_left_lane(
            ) if waypoint_road.lane_change & carla.LaneChange.Left else None
            right_lane = waypoint_road.get_right_lane(
            ) if waypoint_road.lane_change & carla.LaneChange.Right else None
            if left_lane and left_lane.lane_type == waypoint_road.lane_type:
                left_offset = (
                    left_lane.lane_width if left_lane else 0)
            else:
                left_offset = 0

            if right_lane and right_lane.lane_type == waypoint_road.lane_type:
                right_offset = (
                    right_lane.lane_width if right_lane else 0)
            else:
                right_offset = 0
            distance_to_left_side = max(0, lane_width / 2 + left_offset)
            distance_to_right_side = max(0, lane_width / 2 + right_offset)
        else:
            distance_to_left_side = distance_to_right_side = 0.5
        return distance_to_left_side-1, distance_to_right_side-1

    def get_obstacle(self, obs, ego_transform, self_id, ego_speed):
        self.obs_list = []
        if not obs:
            return
        obs_list = []
        for obs_info in obs:
            if obs_info["id"] != self_id and is_within_distance_obs(ego_transform, obs_info, 50, ego_speed, [-65, 65]):
                obs_list.append(
                    [obs_info["location"].x, obs_info["location"].y, obs_info["velocity"], obs_info["yaw"]])
        self.obs_list = obs_list

    def get_nearest_obs(self):
        min_distance = 1000
        nearest_ob = None
        for ob in self.obs_list:
            distance = compute_distance2D(
                (ob[0], ob[1]), [self.location.x, self.location.y])
            if distance < min_distance:
                min_distance = distance
                nearest_ob = ob
        return nearest_ob

    def compute_brake(self, distance):
        brake = math.exp(-distance/10)
        return brake**0.4

    def is_traffic_light_red(self):
        if self.ignore_traffic_light:
            return False
        if self.vehicle.is_at_traffic_light():
            logging.debug("at traffic light")
            traffic_light = self.vehicle.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                return True
        return False

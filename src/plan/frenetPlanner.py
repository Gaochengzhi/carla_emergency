
import time
import functools
import numpy as np
import math
import random
import sys
import carla
from util import compute_distance, is_within_distance_obs, log_time_cost, compute_3D21d, compute_distance2D, compute_magnitude_angle
from view.debug_manager import draw_waypoints, draw_locations
from scipy.interpolate import splprep, splev
import logging


def time_const(fps):
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            target_time_per_frame = 1.0 / fps
            start_time = time.time()
            result = func(*args, **kwargs)
            elapsed_time = time.time() - start_time
            sleep_time = target_time_per_frame - elapsed_time

            if sleep_time > 0:
                time.sleep(sleep_time)
            return result

        return wrapper

    return decorator


class FrenetPlanner():
    def __init__(self, world, map, g_waypoints, vehicle, config, controller):
        self.global_paths = g_waypoints
        self.world = world
        self.map = map
        self.config = config
        self.vehicle = vehicle
        self.self_id = vehicle.id
        self.ob = []
        self.tmp_gpath = []
        self.local_traj = []
        self.tx = []
        self.ty = []
        self.ts = []
        self.left_side = 1.5
        self.right_side = 1.5
        self.c_speed = 0
        self.controller = controller(vehicle)
        self.target_speed = random.randint(20, 25)
        self.location = None

    def update_obstacle(self, obs, ego_transform, self_id, ego_speed):
        obs_list = []
        if not obs:
            return obs_list
        for obs_info in obs:
            if obs_info["id"] != self_id and is_within_distance_obs(ego_transform, obs_info, 60, ego_speed, [-12, 12]):
                obs_list.append(
                    [obs_info["location"].x, obs_info["location"].y])
        self.ob = obs_list

    def _find_nearest_obs(self):
        min_distance = 1000
        nearest_ob = None
        for ob in self.ob:
            distance = compute_distance2D(
                (ob[0], ob[1]), self.location)
            if distance < min_distance:
                min_distance = distance
                nearest_ob = ob
        return nearest_ob

    # @log_time_cost(name="ego_planner")
    def run_step(self, obs, control):
        # get_info
        ego_transform, self.location, self.c_speed, c_accel, c_d, c_d_d, c_d_dd, self.left_side, self.right_side = self.update_vehicle_info()
        self.ob = []
        self.update_obstacle(
            obs, ego_transform, self.self_id, self.c_speed)
        # check
        if len(self.global_paths) < 2:
            control.brake = 0.6
            self.vehicle.apply_control(control)
            logging.info("finish the route!")
            return

            exit()
        if len(self.tmp_gpath) < 1:
            self.update_tgpath()
        if len(self.tx) < 1 or len(self.ty) < 1 or len(self.ts) < 1:
            self.update_tgpath()
            self.update_trajectories()
        if compute_distance(self.tmp_gpath[0].transform.location, self.vehicle.get_location()) < 3:
            self.tmp_gpath.pop(0)
            self.global_paths.pop(0)
        if compute_distance2D((self.tx[0], self.ty[0]), self.location) < 2:
            self.tx = self.tx[1:]
            self.ty = self.ty[1:]
            self.ts = self.ts[1:]
        # Frenet Optimal Trajectory plan
        # # obstacle avoid
        if self.ob:
            ob = self._find_nearest_obs()
            dis = compute_distance2D(ob, self.location)
            if dis < 30:
                control.throttle = 0
                control.brake = 0.5
                # logging.info("obstacle!")
                self.vehicle.apply_control(control)
                return
            if dis < 20:
                control.brake = 1
                # logging.error("obstacle detected!")
                self.vehicle.apply_control(control)
                return

        # DEBUG
        draw_waypoints(self.world, self.tmp_gpath, z=2, life_time=0.3)
        draw_locations(self.world, self.tx, self.ty, size=0.05,
                       color=carla.Color(0, 250, 123), life_time=0.3)
        # run
        try:
            target_waypoint = carla.Transform(
                carla.Location(x=self.tx[0], y=self.ty[0], z=2))
            control = self.controller.run_step(self.ts[0]*3.6, target_waypoint)
            self.vehicle.apply_control(control)
        except Exception as e:
            logging.error(f"plan run_step error: {e}")
            pass

    def update_trajectories(self):
        wx = [point.transform.location.x for point in self.tmp_gpath]
        wy = [point.transform.location.y for point in self.tmp_gpath]
        ws = np.linspace(self.c_speed, self.target_speed, len(wx))
        try:
            wx = [wx[i]
                  for i in range(len(wx)) if i == 0 or wx[i] != wx[i - 1]]
            wy = [wy[i]
                  for i in range(len(wy)) if i == 0 or wy[i] != wy[i - 1]]
            tck, _ = splprep([wx, wy, ws], s=4)
            u_fine = np.linspace(0, 1, 30)
            x_fine, y_fine, s_fine = splev(u_fine, tck)
        except:
            x_fine, y_fine, s_fine = wx, wy, ws
        self.tx = x_fine
        self.ty = y_fine
        self.ts = s_fine

    def update_tgpath(self):
        num_push = min(8, len(self.global_paths) - len(self.tmp_gpath))
        self.tmp_gpath = self.global_paths[:num_push]

    def update_vehicle_info(self):
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        transform = self.vehicle.get_transform()
        acceleration = self.vehicle.get_acceleration()
        waypoint_ego = self.map.get_waypoint(location, project_to_road=False)
        waypoint_road = self.map.get_waypoint(location)
        if waypoint_ego and waypoint_road:
            lane_width = waypoint_road.lane_width
            # Calculate the vehicle's offset from the lane center using yaw and position
            left_lane = waypoint_road.get_left_lane(
            ) if waypoint_road.lane_change & carla.LaneChange.Left else None
            right_lane = waypoint_road.get_right_lane(
            ) if waypoint_road.lane_change & carla.LaneChange.Right else None

            # Calculate distances to the road sides with lateral offset and lane width
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

            # Ensure distances are calculated relative to the vehicle's current lane
            distance_to_left_side = max(0, lane_width / 2 + left_offset)
            distance_to_right_side = max(0, lane_width / 2 + right_offset)
        else:
            distance_to_left_side = distance_to_right_side = 0.5
        c_speed = compute_3D21d(velocity)
        c_accel = compute_3D21d(acceleration)
        c_d = 0.0  # current lateral position [m]
        c_d_d = velocity.y  # current lateral speed [m/s]
        c_d_dd = acceleration.y  # current lateral acceleration [m/s]
        return transform, location, c_speed, c_accel, c_d, c_d_d, c_d_dd, distance_to_left_side, distance_to_right_side

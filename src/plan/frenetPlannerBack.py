
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
        self.obs_list = []
        self.tmp_gpath = []
        self.local_traj = []
        self.speed_traj = []
        self.left_side = 1.5
        self.right_side = 1.5
        self.ego_speed = 0
        self.controller = controller(vehicle)
        self.target_speed = random.randint(23, 26)
        self.location = None

    # @log_time_cost(name="ego_planner")
    def run_step(self, obs, control):
        # get_info
        try:
            ego_transform, self.location, self.ego_speed, _, self.left_side, self.right_side = self.update_vehicle_info()
            self.update_obstacle(
                obs, ego_transform, self.self_id, self.ego_speed)
            # check
            if len(self.global_paths) < 2:
                control.brake = 0.1
                self.vehicle.apply_control(control)
                logging.info("finish the route!")
                # self.vehicle.destroy()

                sys.exit(0)

            if len(self.tmp_gpath) < 2:
                self.update_tgpath()
            if len(self.speed_traj) < 2 or len(self.local_traj) < 2:
                self.update_tgpath()
                self.update_trajectories()
            if compute_distance(self.tmp_gpath[0].transform.location, self.vehicle.get_location()) < 3:
                self.tmp_gpath.pop(0)
                self.global_paths.pop(0)
            if compute_distance2D(self.local_traj[0], self.location) < 2:
                self.local_traj.pop(0)
                self.speed_traj.pop(0)

            if len(self.tmp_gpath) < 2:
                self.update_tgpath()
            if len(self.speed_traj) < 2 or len(self.local_traj) < 2:
                self.update_tgpath()
                self.update_trajectories()
            # Frenet Optimal Trajectory plan
            # # obstacle avoid
            if self.obs_list:
                ob = self._find_nearest_obs()
                dis = compute_distance2D(ob, self.location)
                if dis < 20 and dis > 10:
                    control.throttle = 0
                    control.brake = 0.4
                    # logging.info("obstacle!")
                    self.vehicle.apply_control(control)
                    return
                if dis < 10:
                    control.brake = 0.7
                    # logging.error("obstacle detected!")
                    self.vehicle.apply_control(control)
                    return

            # DEBUG
            draw_waypoints(self.world, self.tmp_gpath, z=2, life_time=0.3)
            draw_list(self.world, self.local_traj, size=0.05,
                      color=carla.Color(0, 250, 123), life_time=0.3)
            # run
            if len(self.local_traj) < 1:
                return
            target_waypoint = carla.Transform(
                carla.Location(x=self.local_traj[0][0], y=self.local_traj[0][1], z=2))
            control = self.controller.run_step(
                self.speed_traj[0]*3.6, target_waypoint)
            self.vehicle.apply_control(control)
        except Exception as e:
            line = sys._getframe().f_lineno
            file = sys._getframe().f_globals["__file__"]
            logging.error(f"ego_planner error:{e} in {file} at {line}")

    def update_trajectories(self):
        xy_list = np.array([[waypoint.transform.location.x, waypoint.transform.location.y]
                            for waypoint in self.tmp_gpath])
        ws = np.linspace(self.ego_speed, self.target_speed, len(xy_list))
        xy_list = np.column_stack([xy_list, ws])
        lenxy = len(xy_list)
        if lenxy > 3:
            tck, u = splprep(xy_list.T, s=4)
        elif lenxy < 4 and lenxy > 1:
            tck, u = splprep(xy_list.T, s=4, k=1)
        else:
            self.local_traj = xy_list.tolist()
            self.speed_traj = ws.tolist()
            return

        u_new = np.linspace(u.min(), u.max(), 30)
        x_fine, y_fine, s_fine = splev(u_new, tck)
        interpolated_waypoints = np.column_stack([x_fine, y_fine]).tolist()
        self.speed_traj = s_fine.tolist()
        self.local_traj = interpolated_waypoints

    def update_tgpath(self):
        num_push = min(10, len(self.global_paths))
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

            distance_to_left_side = max(0, lane_width / 2 + left_offset)
            distance_to_right_side = max(0, lane_width / 2 + right_offset)
        else:
            distance_to_left_side = distance_to_right_side = 0.5
        c_speed = compute_3D21d(velocity)
        c_accel = compute_3D21d(acceleration)
        return transform, location, c_speed, c_accel,  distance_to_left_side, distance_to_right_side

    def update_obstacle(self, obs, ego_transform, self_id, ego_speed):
        self.obs_list = []
        if not obs:
            return
        obs_list = []
        for obs_info in obs:
            if obs_info["id"] != self_id and is_within_distance_obs(ego_transform, obs_info, 60, ego_speed, [-10, 10]):
                obs_list.append(
                    [obs_info["location"].x, obs_info["location"].y, obs_info["velocity"], obs_info["yaw"]])
        self.obs_list = obs_list

    def _find_nearest_obs(self):
        min_distance = 1000
        nearest_ob = None
        for ob in self.obs_list:
            distance = compute_distance2D(
                (ob[0], ob[1]), self.location)
            if distance < min_distance:
                min_distance = distance
                nearest_ob = ob
        return nearest_ob

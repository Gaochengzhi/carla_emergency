
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
    def __init__(self, world, map, router_waypoints, vehicle, config, controller):
        self.global_waypoints = router_waypoints
        self.world = world
        self.map = map
        self.config = config
        self.vehicle = vehicle
        self.self_id = vehicle.id
        self.obs_list = []
        self.waypoint_buffer = []
        self.local_traj = []
        self.offset = 0
        self.left_side = 1.5
        self.right_side = 1.5
        self.ego_speed = 0
        self.acceleration = 0
        self.step = 0
        self.transform = None
        self.controller = controller(vehicle)
        self.car_following_model = IDM()
        self.target_speed = random.randint(95, 100)
        self.location = None
        self.use_car_following = False
        self.vehicle.show_debug_telemetry(True)
        self.ignore_traffic_light = config["ignore_traffic_light"]
        self.get_vehicle_info()
        self.check_update_waypoints()

    @log_time_cost(name="ego_planner")
    def run_step(self, obs, control):
        # get_info
        try:
            self.get_vehicle_info()
            self.get_obstacle(
                obs, self.transform, self.self_id, self.ego_speed)
            # check finish
            if len(self.global_waypoints) < 1:
                self.vehicle.destroy()
                logging.info("finish the route!")
                exit()

            if compute_distance(self.waypoint_buffer[0].transform.location, self.vehicle.get_location()) < 5+abs(self.offset):
                self.waypoint_buffer.pop(0)
                self.global_waypoints.pop(0)
                self.update_waypoint_buffer(self.ego_speed)
                # self.update_trajectories(self.offset)
                # self.check_collison(self.obs_list, self.local_traj)
            # check collison
            if len(self.local_traj) < 1:
                return

            if compute_distance2D(self.local_traj[0], [self.location.x, self.location.y]) < 5:
                self.local_traj.pop(0)
                self.update_trajectories(self.offset)
            if self.check_collison(self.obs_list, self.local_traj) and self.step % 10 == 0:
                self.adjust_trajectories(self.obs_list)
            if len(self.local_traj) < 1:
                return
            # Frenet Optimal Trajectory plan
            # DEBUG
            draw_waypoints(self.world, self.waypoint_buffer,
                           z=2, life_time=0.3, size=0.2)
            # draw_list(self.world, self.local_traj, size=0.2,
            #           color=carla.Color(0, 250, 123), life_time=0.25)

            # CONTROLLER
            target_waypoint = carla.Transform(
                carla.Location(x=self.local_traj[0][0], y=self.local_traj[0][1], z=1))
            control = self.controller.run_step(
                self.target_speed, target_waypoint)
            self.vehicle.apply_control(control)
            self.step += 1
        except Exception as e:
            logging.error(f"plan run_step error: {e}")
            print(e.__traceback__.tb_frame.f_globals["__file__"])
            print(e.__traceback__.tb_lineno)
            pass

    def adjust_trajectories(self, ob_list):
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
            self.update_trajectories(self.offset)

    # @log_time_cost(name="update_trajectories")
    def update_trajectories(self, offset=0):
        xy_list = [[waypoint.transform.location.x, waypoint.transform.location.y]
                   for waypoint in self.waypoint_buffer]

        # xy_list.insert(0, [self.location.x, self.location.y])
        xy_list = [xy_list[i] for i in range(
            len(xy_list)) if i == 0 or xy_list[i] != xy_list[i-1]]

        lenxy = len(xy_list)
        xy_list = np.array(xy_list)
        # Interpolation setup
        if lenxy > 3:
            tck, u = splprep(xy_list.T, s=4, k=3)
        elif lenxy > 1:
            tck, u = splprep(xy_list.T, s=4, k=1)
        else:
            self.local_traj = xy_list.tolist()
            return

        u_new = np.linspace(u.min(), u.max(), lenxy * 3)
        x_fine, y_fine = splev(u_new, tck)
        interpolated_waypoints = np.column_stack([x_fine, y_fine])

        if offset != 0:
            # Compute directions and normals for offset application
            directions = np.diff(interpolated_waypoints, axis=0)
            normals = np.array([-directions[:, 1], directions[:, 0]]).T
            # Normalize
            normals = (normals.T / np.linalg.norm(normals, axis=1)).T
            offset_vectors = normals * offset
            # Duplicate first vector for dimensions match
            offset_vectors = np.vstack([[offset_vectors[0]], offset_vectors])

            # Apply offset
            interpolated_waypoints += offset_vectors

        self.local_traj = interpolated_waypoints.tolist()

    def check_collison(self, obs, traject_points):
        def interpolate_points(start, end, spacing=2.5):
            distance = compute_distance2D(start, end)
            num_points = max(int(distance / spacing), 2)
            x_spacing = (end[0] - start[0]) / (num_points - 1)
            y_spacing = (end[1] - start[1]) / (num_points - 1)
            return [(start[0] + i * x_spacing, start[1] + i * y_spacing) for i in range(num_points)]

        ob_list = []
        start_location = (self.location.x, self.location.y)
        end_location = traject_points[0]
        # Interpolate between self.location and the first trajectory point
        interpolated_points = interpolate_points(start_location, end_location)
        # Combine interpolated points with the rest of the trajectory points
        traject_points = interpolated_points + traject_points
        draw_list(self.world, traject_points, size=0.2,
                  color=carla.Color(210, 250, 123), life_time=0.25)

        for ob in obs:
            for point in traject_points:
                if compute_distance2D((point[0], point[1]), (ob[0], ob[1])) < 2:
                    ob_list.append(ob)
                    break
        return ob_list

    def update_waypoint_buffer(self, speed):
        num_push = min(int(speed*2/5)+1, len(self.global_waypoints))
        self.waypoint_buffer = self.global_waypoints[:num_push]

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
        # print(distance_to_left_side, distance_to_right_side)
        return distance_to_left_side-0.2, distance_to_right_side-0.2

    def get_obstacle(self, obs, ego_transform, self_id, ego_speed):
        self.obs_list = []
        if not obs:
            return
        obs_list = []
        for obs_info in obs:
            if obs_info["id"] != self_id and is_within_distance_obs(ego_transform, obs_info, 60, ego_speed, [-65, 65]):
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

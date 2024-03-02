
import time
import functools
import numpy as np
import math
import random
import sys
import carla
from util import compute_distance, is_within_distance_obs, log_time_cost, compute_3D21d, compute_distance2D, compute_magnitude_angle, interpolate_points
from view.debug_manager import draw_waypoints, draw_list
from scipy.interpolate import splprep, splev
from plan.carFollowingModel import IDM
from perception.perception_basline import FakePerception
import logging
from pyinstrument import Profiler


class FrenetPlanner():
    def __init__(self, world, map, router_waypoints, vehicle, config, controller, sensor_manager):
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
        self.ego_speed = 0
        self.acceleration = 0
        self.step = 0
        self.transform = None
        self.controller = controller
        self.sensor_manager = sensor_manager
        self.perception = FakePerception(vehicle, config)
        self.car_following_model = IDM()
        self.use_car_following = False
        self.max_speed = 33
        self.target_speed = 20
        self.location = None
        self.vehicle.show_debug_telemetry(True)
        self.ignore_traffic_light = config["ignore_traffic_light"]
        self.check_update_waypoints()
        self.left_side, self.right_side = self.perception.get_road_edge(
            self.waypoint_buffer[-1])
        self.profiler = Profiler(interval=0.001)

    # @log_time_cost(name="ego_planner")
    def run_step(self, obs, control):
        # get_info
        try:
            self.profiler.start()
            # check finish
            self.update_info()
            self.get_obstacle(
                obs, self.transform, self.self_id, self.ego_speed)

            if compute_distance(self.waypoint_buffer[0].transform.location, self.location) < 5+abs(int(self.offset)):
                self.waypoint_buffer.pop(0)
                self.global_waypoints.pop(0)
                if len(self.global_waypoints) < 1:
                    self.vehicle.destroy()
                    logging.info("finish the route!")
                    exit()
                self.update_waypoint_buffer()
                self.left_side, self.right_side = self.perception.get_road_edge(
                    self.waypoint_buffer[-1])
                # self.update_trajectories(self.offset)
            # check collison
            if len(self.local_traj) < 1:
                return

            if compute_distance2D(self.local_traj[0], [self.location.x, self.location.y]) < 5:
                self.local_traj.pop(0)
                self.update_trajectories(self.offset)

            if len(self.local_traj) < 1:
                return
            if self.step % 8 == 0:
                if self.check_collison(self.obs_list, self.local_traj):
                    self.adjust_trajectories()
                else:
                    if self.step % 16 == 0:
                        if self.offset > 0.1:
                            self.offset -= 0.1
                        elif self.offset < -0.1:
                            self.offset += 0.1
                    # print(self.offset)

            if len(self.local_traj) < 1:
                return
            # Frenet Optimal Trajectory plan
            # DEBUG
            draw_waypoints(self.world, self.waypoint_buffer,
                           z=2, life_time=0.3, size=0.2)
            draw_list(self.world, self.local_traj, size=0.2,
                      color=carla.Color(0, 250, 123), life_time=0.25)
            if self.use_car_following:
                ob = self.sensor_manager.obstacle
                if ob:
                    front_v = ob.velocity
                    distance_s = ob.distance
                    a = self.car_following_model.calc_acc(
                        front_v, distance_s, self.ego_speed)
                    self.target_speed = max(0, self.ego_speed + a)

            # CONTROLLER
            target_waypoint = carla.Transform(
                carla.Location(x=self.local_traj[0][0], y=self.local_traj[0][1], z=1))
            control = self.controller.run_step(
                self.target_speed, target_waypoint)
            self.vehicle.apply_control(control)
            self.step += 1
            self.profiler.stop()
            # self.profiler.print()

        except Exception as e:
            logging.error(f"plan run_step error: {e}")
            logging.error(e.__traceback__.tb_frame.f_globals["__file__"])
            logging.error(e.__traceback__.tb_lineno)
            logging.error(self.obs_list)
            pass

    def adjust_trajectories(self):

        # Negative, starting close to zero
        left_options = np.arange(-1, -self.left_side - 1, -1)
        right_options = np.arange(1, self.right_side + 1, 1)
        traj_adjust_options = np.concatenate(
            ([0], left_options, right_options))
        # Store tuples of (offset, highest_velocity, nearest_index)
        collision_info = []

        for offset in traj_adjust_options:
            # Apply offset to calculate the new trajectory without modifying the original yet
            self.update_trajectories(offset)
            collision_result = self.check_collison(
                self.obs_list, self.local_traj)

            if collision_result:
                # Find the highest velocity among collisions for this offset
                highest_velocity = max(collision_result, key=lambda x: x[1])[1]
                nearest_index = min(collision_result, key=lambda x: x[2])[2]
                collision_info.append(
                    (offset, highest_velocity, nearest_index))
            else:
                # Found a collision-free offset, apply it and return
                self.offset = offset
                return  # Exit as we've applied a suitable adjustment

        # If here, no collision-free offset was found; decide based on obstacle velocities and distances
        if collision_info:
            # Choose the offset related to the nearest highest-velocity obstacle
            # chosen_offset = max(collision_info, key=lambda x: (x[2], x[1]))[0]
            chosen_offset = 0
            self.offset = chosen_offset
            self.update_trajectories(chosen_offset)
            self.use_car_following = True

    def check_update_waypoints(self):
        if len(self.waypoint_buffer) < 1:
            self.update_waypoint_buffer()
        if len(self.local_traj) < 1:
            self.update_waypoint_buffer()
            self.update_trajectories(self.offset)

    # @log_time_cost(name="update_trajectories")
    def update_trajectories(self, offset=0):
        xy_list = [[waypoint.transform.location.x, waypoint.transform.location.y]
                   for waypoint in self.waypoint_buffer]

        # xy_list.insert(0, [self.location.x, self.location.y])
        # xy_list = [xy_list[i] for i in range(
        #     len(xy_list)) if i == 0 or xy_list[i] != xy_list[i-1]]

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

        u_new = np.linspace(u.min(), u.max(), lenxy * 2)
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
        start_location = [self.location.x, self.location.y]
        end_location = traject_points[0]
        interpolated_points = interpolate_points(
            start_location, end_location, 2)
        traject_points = interpolated_points + traject_points
        # debug
        draw_list(self.world, traject_points, size=0.1,
                  color=carla.Color(110, 25, 144), life_time=0.15)
        collision_info = []
        for ob in obs:
            for index, point in enumerate(traject_points):
                if compute_distance2D((point[0], point[1]), (ob[0], ob[1])) < 2.3:
                    collision_info.append((ob, ob[2], index))
                    break
        if collision_info:
            return collision_info
        else:
            self.use_car_following = False
            self.target_speed = self.max_speed
            return False

    def update_waypoint_buffer(self):
        num_push = min(int(self.ego_speed*2/5)+2,
                       len(self.global_waypoints), 13)
        self.waypoint_buffer = self.global_waypoints[:num_push]
        if num_push < 3:
            return
        curvature = 0
        for i in range(1, num_push - 1):
            x1, y1 = self.waypoint_buffer[i -
                                          1].transform.location.x, self.waypoint_buffer[i-1].transform.location.y
            x2, y2 = self.waypoint_buffer[i].transform.location.x, self.waypoint_buffer[i].transform.location.y
            x3, y3 = self.waypoint_buffer[i +
                                          1].transform.location.x, self.waypoint_buffer[i+1].transform.location.y
            # Vector from the first to the second point
            dx1, dy1 = x2 - x1, y2 - y1
            # Vector from the second to the third point
            dx2, dy2 = x3 - x2, y3 - y2
            # Normalize vectors
            mag1 = (dx1**2 + dy1**2)**0.5
            mag2 = (dx2**2 + dy2**2)**0.5
            if mag1 > 0 and mag2 > 0:
                dx1, dy1 = dx1 / mag1, dy1 / mag1
                dx2, dy2 = dx2 / mag2, dy2 / mag2
                # Dot product between vectors (cosine of the angle)
                dot = dx1*dx2 + dy1*dy2
                # Update curvature (using 1 - dot to get a measure of deviation from straight line)
                curvature += (1 - dot)
        if curvature > 0.0001:
            self.target_speed = max(
                12, self.max_speed * math.exp(-10*curvature))
        else:
            self.target_speed = self.max_speed

    def get_obstacle(self, obs, ego_transform, self_id, ego_speed):
        self.obs_list = []
        if not obs:
            return
        obs_list = []
        ego_location = [ego_transform.location.x, ego_transform.location.y]
        ego_yaw = ego_transform.rotation.yaw
        for obs_info in obs:
            if obs_info["id"] != self_id:
                target_location = [
                    obs_info["location"].x, obs_info["location"].y]
                target_velocity = obs_info["velocity"]

                if is_within_distance_obs(ego_location, target_location, 60, ego_speed, target_velocity, ego_yaw, [-65, 65]):
                    obs_list.append(
                        [obs_info["location"].x, obs_info["location"].y, obs_info["velocity"], obs_info["yaw"]])

        self.obs_list = obs_list

    def compute_brake(self, distance):
        brake = math.exp(-distance/10)
        return brake**0.4

    def update_info(self):
        self.location, self.transform, self.ego_speed, self.acceleration = self.perception.get_vehicle_info()

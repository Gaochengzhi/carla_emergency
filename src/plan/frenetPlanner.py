
import time
import functools
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import carla
from lib.quintic_polynomials_planner import QuinticPolynomial
from lib import cubic_spline_planner
from util import compute_distance, is_within_distance_obs, log_time_cost,compute_3D21d, compute_distance2D
from view.debug_manager import draw_waypoints, draw_locations
from scipy.interpolate import splprep, splev


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
    def __init__(self,world, g_waypoints ,vehicle, config, controller):
        self.global_paths = g_waypoints
        self.world = world
        self.config = config
        self.vehicle = vehicle
        self.self_id = vehicle.id
        self.ob = []
        self.tmp_gpath = []
        self.local_traj = []
        self.csp = []
        self.tx = []
        self.ty = []
        self.controller = controller(vehicle)
        self.path = None
        self.s0 = 0

    def update_obstacle(self,obs,ego_transform, self_id, ego_speed):
        obs_list = []
        if not obs:
            return obs_list
        for obs_info in obs:
            if obs_info["id"] != self_id and is_within_distance_obs(ego_transform,obs_info,80,ego_speed,[-60,60]):
                obs_list.append([obs_info["location"].x,obs_info["location"].y])
        return np.array(obs_list)
                

    @log_time_cost
    def run_step(self,obs, control):
        if len(self.global_paths)<2:
            control.brake = 1.0
            self.vehicle.apply_control(control)
            return
        if len(self.tmp_gpath)<2:
            self.update_tmp_gpath()

        ego_transform,c_location,c_speed, c_accel, c_d,c_d_d,c_d_dd ,s0 = self.update_vehicle_info()
        self.ob = self.update_obstacle(obs,ego_transform,self.self_id, c_speed)
        ## DEBUG
        draw_waypoints(self.world, self.tmp_gpath, z=2, life_time=1)
        draw_locations(self.world, self.tx,self.ty,color=carla.Color(0,250,123),life_time=0.2 )

        if compute_distance(self.tmp_gpath[0].transform.location,self.vehicle.get_location())<3:
            self.tmp_gpath.pop(0)
            self.global_paths.pop(0)
        
        if len(self.tx)<2:
            self.update_tmp_gpath()


        if compute_distance2D((self.tx[0], self.ty[0]), c_location)<2:
                self.tx = self.tx[1:]
                self.ty = self.ty[1:]
        try:
            target_waypoint  = carla.Transform(carla.Location(x=self.tx[0], y=self.ty[0], z=2))
            control = self.controller.run_step(20, target_waypoint)
            self.vehicle.apply_control(control)
        except:
            pass

    def update_tmp_gpath(self):
        num_push = min(6, len(self.global_paths) - len(self.tmp_gpath))
        self.tmp_gpath = self.global_paths[:num_push]
        wx = [point.transform.location.x for point in self.tmp_gpath]
        wy = [point.transform.location.y for point in self.tmp_gpath]
        # filter out the identical points of wy and wx
        try:
            wx = [wx[i] for i in range(len(wx)) if i == 0 or wx[i] != wx[i - 1]]
            wy = [wy[i] for i in range(len(wy)) if i == 0 or wy[i] != wy[i - 1]]
            tck, _ = splprep([wx, wy], s=4)
            u_fine = np.linspace(0, 1, 30)
            x_fine, y_fine, = splev(u_fine, tck)
        except:
            x_fine, y_fine = wx, wy
        self.tx = x_fine
        self.ty = y_fine

    def update_vehicle_info(self):
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        transform = self.vehicle.get_transform()
        acceleration = self.vehicle.get_acceleration()
        c_speed = compute_3D21d(velocity)
        c_accel = compute_3D21d(acceleration)
        c_d = 0.0  # current lateral position [m]
        c_d_d = velocity.y  # current lateral speed [m/s]
        c_d_dd = acceleration.y  # current lateral acceleration [m/s]
        return transform,location,c_speed, c_accel, c_d,c_d_d,c_d_dd, self.s0

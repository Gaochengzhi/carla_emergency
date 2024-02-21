import time
import functools
import logging
from colorlog import ColoredFormatter
import numpy as np
import os
import random
# from tools.config_manager import config
import carla
import math
import csv

class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

def txt_to_points(line):
    # line is like "a,b,c,d,e,f"
    sp = carla.Transform(carla.Location(float(line[0]), float(line[1]), float(line[2])), carla.Rotation(float(line[3]), float(line[4]), float(line[5])))
    return sp

    
def load_sppoints_from_csv(filepath):
    wp_list = []
    with open(filepath, "r") as file:
        reader = csv.reader(file) 
        for line in reader:
            wp_list.append(txt_to_points(line))
    return wp_list

def connect_to_server(config):
    carla_timeout = config["carla_timeout"]
    carla_port = config["carla_port"]
    client = carla.Client("localhost", carla_port)
    client.set_timeout(carla_timeout)
    world = client.get_world()
    return client, world


def destroy_all_actors(world):
    for actor in world.get_actors():
        if actor.type_id.startswith("vehicle") or actor.type_id.startswith("sensor"):
            actor.destroy()
    world.tick()
    


    


def spawn_vehicle(world, vehicle_type, spawn_point, hero=False):
    lz = spawn_point.location.z + 0.5
    spawn_point= carla.Transform(carla.Location(spawn_point.location.x, spawn_point.location.y, lz), spawn_point.rotation)
    vehicle_bp = world.get_blueprint_library().filter(
        vehicle_type
    )[0]
    if hero:
        vehicle_bp.set_attribute('role_name', 'hero')
    vehicle = world.try_spawn_actor(
        vehicle_bp, spawn_point)
    return vehicle


def waypoints_center(waypoint_list):
    x = []
    y = []
    z = []
    for waypoint in waypoint_list:
        x.append(waypoint.transform.location.x)
        y.append(waypoint.transform.location.y)
        z.append(waypoint.transform.location.z)
    return carla.Location(
        np.mean(x), np.mean(y), np.mean(z)
    )




def get_ego_vehicle(world):
    actor = None
    while not actor:
        for actor in world.get_actors():
            if actor.type_id.startswith("vehicle"):
                if actor.attributes["role_name"] == "hero":
                    return actor
        time.sleep(0.2)
    raise RuntimeError("No ego vehicle found")

def log_time_cost(func):
    """
    Decorator to log the execution time of a function.
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()  # Start time of function execution
        result = func(*args, **kwargs)  # Execute the function
        elapsed_time = time.time() - start_time  # Calculate elapsed time
        
        # Log the time cost with debug level
        logging.debug(f"Function {func.__name__} executed in {elapsed_time:.4f} seconds.")
        
        return result
    return wrapper

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

# turn carla way point into NetworkX graph point


def waypoint_to_graph_point(waypoint):
    return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)


def batch_process_vehicles(world, ego,  max_distance, angle, func, *args, **kwargs):

    vehicles = []
    for actor in world.get_actors():
        if actor.type_id.startswith("vehicle") and actor.attributes["role_name"] != "hero":
            if is_within_distance(actor.get_transform(), ego.get_transform(), max_distance, angle_interval=angle):
                processed_vehicle = func(
                    world, actor, ego, *args, **kwargs)
                vehicles.append(processed_vehicle)
    return vehicles


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def get_trafficlight_trigger_location(traffic_light):
    """
    Calculates the yaw of the waypoint that represents the trigger volume of the traffic light
    """
    def rotate_point(point, radians):
        """
        rotate a given point by a given angle
        """
        rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
        rotated_y = math.sin(radians) * point.x - math.cos(radians) * point.y

        return carla.Vector3D(rotated_x, rotated_y, point.z)

    base_transform = traffic_light.get_transform()
    base_rot = base_transform.rotation.yaw
    area_loc = base_transform.transform(traffic_light.trigger_volume.location)
    area_ext = traffic_light.trigger_volume.extent

    point = rotate_point(carla.Vector3D(
        0, 0, area_ext.z), math.radians(base_rot))
    point_location = area_loc + carla.Location(x=point.x, y=point.y)

    return carla.Location(point_location.x, point_location.y, point_location.z)


def is_within_distance(target_transform, reference_transform, max_distance, angle_interval=None):
    """
    Check if a location is both within a certain distance from a reference object.
    By using 'angle_interval', the angle between the location and reference transform
    will also be tkaen into account, being 0 a location in front and 180, one behind.

    :param target_transform: location of the target object
    :param reference_transform: location of the reference object
    :param max_distance: maximum allowed distance
    :param angle_interval: only locations between [min, max] angles will be considered. This isn't checked by default.
    :return: boolean
    """
    target_vector = np.array([
        target_transform.location.x - reference_transform.location.x,
        target_transform.location.y - reference_transform.location.y
    ])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    # Further than the max distance
    if norm_target > max_distance:
        return False

    # We don't care about the angle, nothing else to check
    if not angle_interval:
        return True

    min_angle = angle_interval[0]
    max_angle = angle_interval[1]

    fwd = reference_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    angle = math.degrees(math.acos(
        np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return min_angle < angle < max_angle


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array(
        [target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(
        np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    """
    Returns the 2D distance from a waypoint to a vehicle

        :param waypoint: actual waypoint
        :param vehicle_transform: transform of the target vehicle
    """
    loc = vehicle_transform.location
    x = waypoint.transform.location.x - loc.x
    y = waypoint.transform.location.y - loc.y

    return math.sqrt(x * x + y * y)


def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2

        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


def compute_distance(location_1, location_2):
    """
    Euclidean distance between 3D points

        :param location_1, location_2: 3D points
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
    return norm


def positive(num):
    """
    Return the given number if positive, else 0

        :param num: value to check
    """
    return num if num > 0.0 else 0.0
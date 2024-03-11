import carla
import time
import sys
import logging
import os
import random
from util import load_points_from_csv, spawn_vehicle, get_ego_vehicle


import threading


class WorldManager:
    def __init__(self, config):
        self.config = config
        self.client = self.create_client()
        self.world = self.create_world()
        self.map = self.world.get_map()
        self.traffic_agent = self.set_traffic_agent()
        self.set_weather()
        self.set_world_parameter()
        # self.start_init_traffic_flow_thread()

    # def start_init_traffic_flow_thread(self):
    #     thread = threading.Thread(target=self.init_traffic_flow_thread)
    #     thread.start()

    # def init_traffic_flow_thread(self):
    #     time.sleep(2)
    #     self.init_traffic_flow(self.world, self.client)

    def choose_a_point(self, waypoint_list):
        choosen_waypoint = random.choice(waypoint_list)
        waypoint_list.remove(choosen_waypoint)
        return choosen_waypoint

    def set_traffic_agent(self):
        traffic_agent = self.client.get_trafficmanager()
        traffic_agent.set_synchronous_mode(False)
        traffic_agent.set_hybrid_physics_radius(
            self.config["hybrid_physics_radius"])
        # for larger map
        traffic_agent.set_respawn_dormant_vehicles(True)
        traffic_agent.set_boundaries_respawn_dormant_vehicles(50, 800)
        return traffic_agent

    def create_client(self):
        client = carla.Client("localhost", self.config["carla_port"])
        client.set_timeout(self.config["carla_timeout"])
        return client

    def _compare_map(self, map_name, world):
        return str(world.get_map().name)[-2:] != map_name[-2:]

    def create_world(self):
        # print all available maps
        all_maps = self.client.get_available_maps()
        map_name = self.config["map_name"]
        if self.config["is_custum_map"]:
            data = None
            # with open("/home/ujs/mycode/highways.osm", encoding='utf-8') as od_file:
            with open("/home/ujs/mycode/carla_emergency/assets/IntersectionA.osm", encoding='utf-8') as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    print('file could not be readed.')
                    sys.exit()
            logging.info('Converting OSM data to opendrive')
            xodr_data = carla.Osm2Odr.convert(data)
            logging.info('load opendrive map.')
            vertex_distance = 8.0  # in meters
            max_road_length = 9500.0  # in meters
            wall_height = 0.5      # in meters
            extra_width = 1.6      # in meters
            world = self.client.generate_opendrive_world(
                xodr_data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True))
            return world

        current_world = self.client.get_world()
        if (
            self._compare_map(map_name, current_world)
        ):
            return self.client.load_world(map_name)
        else:
            return current_world

    def set_weather(self):
        clear_weather = carla.WeatherParameters(
            cloudiness=self.config["cloudiness"],
            precipitation=self.config["precipitation"],
            sun_altitude_angle=self.config["sun_altitude_angle"],
            sun_azimuth_angle=self.config["sun_azimuth_angle"],
        )
        self.world.set_weather(clear_weather)

    def set_world_parameter(self):
        settings = self.world.get_settings()
        settings.actor_active_distance = self.config["actor_active_distance"]
        settings.synchronous_mode = False
        settings.no_rendering_mode = self.config["no_rendering_mode"]
        settings.tile_stream_distance = self.config["tile_stream_distance"]
        settings.fixed_delta_seconds = self.config["fixed_delta_seconds"]
        self.world.apply_settings(settings)

    # def init_traffic_flow(self, world, client):
    #     spawn_points_distance = self.config["spwan_points_distance"]
    #     area_limit = self.config["area_limit"]
    #     num_vehicles = self.config["num_vehicles"]
    #     v_types = self.config["bg_vehicle_type"]
    #     filtered_points = self._gen_filtered_points(
    #         area_limit, spawn_points_distance
    #     )
    #     if self.config["debug_sp"]:
    #         from view.debug_manager import draw_transforms_with_index
    #         draw_transforms_with_index(
    #             self.world, filtered_points, life_time=1000)
    #     _tm = self.traffic_agent
    #     self.create_bg_vehicles(
    #         world, num_vehicles, v_types, filtered_points, _tm)

    def _gen_filtered_points(self, area_limit, sp_distance):
        map_name = self.map.name.split("/")[-1]
        cache_dir = "cache/sp_points"
        filename = f"{map_name}.csv"
        filepath = os.path.join(cache_dir, filename)
        if os.path.exists(filepath) and False:
            return load_points_from_csv(filepath)
        else:
            distance_sps = self.map.get_spawn_points()
            filtered_points = []
            for p in distance_sps:
                p_location = p.location
                if (
                    area_limit["min_x"] <= p_location.x <= area_limit["max_x"]
                    and area_limit["min_y"] <= p_location.y <= area_limit["max_y"]
                ):
                    filtered_points.append(carla.Transform(
                        p_location, p.rotation))

            os.makedirs(cache_dir, exist_ok=True)
            with open(filepath, "w") as file:
                for sp in filtered_points:
                    file.write(
                        f"{sp.location.x},{sp.location.y},{sp.location.z},{sp.rotation.yaw},{sp.rotation.pitch},{sp.rotation.roll}\n")
            return filtered_points

    # def create_bg_vehicles(self, world, num_vehicles, vehicle_types, spawn_points, traffic_agent):
    #     bg_vehicles = []
    #     for _ in range(num_vehicles):
    #         try:
    #             spawn_point = self.choose_a_point(spawn_points)
    #             v_type = random.choice(vehicle_types)
    #             vehicle = spawn_vehicle(world,
    #                                     v_type, spawn_point)
    #             while vehicle is None:
    #                 logging.warn(
    #                     f"spawn_actor{v_type} failed, trying another start point...")
    #                 v_type = random.choice(vehicle_types)
    #                 spawn_point = self.choose_a_point(spawn_points)
    #                 vehicle = spawn_vehicle(world, v_type, spawn_point)
    #             if vehicle is not None:
    #                 vehicle.set_autopilot(True, traffic_agent.get_port())
    #             bg_vehicles.append(vehicle)
    #         except Exception as e:
    #             logging.error(f"create traffic flow error:{e}")
    #     logging.debug(
    #         f"spawned {len(bg_vehicles)} vehicles")
    #     return bg_vehicles

    def get_map(self):
        return self.map

    def get_world(self):
        return self.world

    def get_client(self):
        return self.client

    def get_traffic_manager(self):
        return self.traffic_agent

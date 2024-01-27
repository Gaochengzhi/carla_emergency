import carla
import time


class WorldManager:
    def __init__(self, config):
        carla_port = config["PortParameters"]["carla_port"]
        area_limit = config["TrafficParameters"]["area_limit"]
        carla_timeout = config["WorldParameters"]["carla_timeout"]
        map_name = config["WorldParameters"]["map_name"]
        spawn_points_distance = config["TrafficParameters"]["spwan_points_distance"]
        world_config = config["WorldParameters"]

        self.client = self.create_client(
            carla_port, carla_timeout)
        self.world = self.create_world(map_name)
        self.map = self.world.get_map()
        self.filtered_waypoints = self.generate_filtered_waypoints(
            area_limit, spawn_points_distance
        )
        self.traffic_agent = self.set_traffic_agent(world_config)
        self.set_weather(config["WeatherParameters"])
        self.set_world_parameter(world_config)

    def set_traffic_agent(self, world_config):
        traffic_agent = self.client.get_trafficmanager()
        traffic_agent.set_synchronous_mode(True)
        traffic_agent.set_hybrid_physics_radius(
            world_config["hybrid_physics_radius"])
        # for larger map
        traffic_agent.set_respawn_dormant_vehicles(True)
        traffic_agent.set_boundaries_respawn_dormant_vehicles(50,800)
        return traffic_agent

    def create_client(self, carla_port, carla_timeout):
        client = carla.Client("localhost", carla_port)
        client.set_timeout(carla_timeout)
        return client

    def compare_map(self, map_name, world):
        return str(world.get_map().name)[-2:] != map_name[-2:]

    def create_world(self, map_name):
        current_world = self.client.get_world()
        if (
            self.compare_map(map_name, current_world)
        ):
            return self.client.load_world(map_name)
        else:
            return current_world

    def set_weather(self, weather_config):
        clear_weather = carla.WeatherParameters(
            cloudiness=weather_config["cloudiness"],
            precipitation=weather_config["precipitation"],
            sun_altitude_angle=weather_config["sun_altitude_angle"],
            sun_azimuth_angle=weather_config["sun_azimuth_angle"],
        )
        self.world.set_weather(clear_weather)

    def set_world_parameter(self, world_config):
        settings = self.world.get_settings()
        settings.actor_active_distance = world_config["actor_active_distance"]
        settings.synchronous_mode = True
        settings.tile_stream_distance = world_config["tile_stream_distance"]
        settings.fixed_delta_seconds = 1.0 / world_config["frame_rate"]
        self.world.apply_settings(settings)

    def generate_filtered_waypoints(self, area_limit, waypoint_distance=20.0):
        distance_waypoints = self.map.generate_waypoints(waypoint_distance)
        filtered_waypoints = []
        for waypoint in distance_waypoints:
            waypoint_location = waypoint.transform.location
            if (
                area_limit["min_x"] <= waypoint_location.x <= area_limit["max_x"]
                and area_limit["min_y"] <= waypoint_location.y <= area_limit["max_y"]
            ):
                filtered_waypoints.append(waypoint)

        return filtered_waypoints

    def get_filtered_waypoints(self):
        return self.filtered_waypoints

    def get_trafficmanager(self):
        return self.traffic_agent

    def get_map(self):
        return self.map

    def get_world(self):
        return self.world

    def get_client(self):
        return self.client

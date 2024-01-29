import carla
import math


class DebugManager:
    def __init__(self, world, urban_waypoints, config):
        self.world = world
        self.debug = world.debug
        # self.draw_global_waypoint = config["draw_global_waypoint"]
        # self.draw_trajectory = config["draw_trajectory"]
        # if config["DebugParameters"]["draw_spawn_points"]:
        #     self.draw_waypoints(world, urban_waypoints)
        # pass
        # self.set_bird_view(world, urban_waypoints, config)


def update_view(world, x, y, z, rotation=carla.Rotation(-90, 0, 0)):
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            carla.Location(x, y, z),
            rotation
        )
    )


def set_bird_view(world, location, config):
    update_view(
        world,
        location.x,
        location.y,
        location.z + config["DebugParameters"]["camera_height"]
    )


def draw_waypoints_arraw(world, waypoints, z=2, color=carla.Color(255, 0, 0), life_time=0.1):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=life_time)


def draw_waypoints(world, waypoints, z=2, color=carla.Color(255, 0, 0), size=0.09, life_time=0.1):
    """
    draw a list of waypoints
    """
    for waypoint in waypoints:
        world.debug.draw_point(
            waypoint.transform.location, size=size, color=color, life_time=life_time
        )


def draw_strings(world, strings, location, color=carla.Color(255, 0, 0), life_time=0.09):
    for string in strings:
        world.debug.draw_string(
            location, string, draw_shadow=False, color=color, life_time=life_time
        )

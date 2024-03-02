import carla
import numpy as np
import time
import logging
import weakref
from util import compute_3D21d


class SensorManager:
    def __init__(self, world, vehicle, config):
        self.world = world
        self.vehicle = vehicle
        # self.add_radar()
        self.add_obstacle_sensor()
        # self.add_camera()
        self.camera_queue = []
        self.radar_queue = []
        self.obstacle = None

        # self.add_camera()

    def add_camera(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '192')
        camera_bp.set_attribute('image_size_y', '108')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

        self.camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.camera_callback)

    def camera_callback(self, data):
        data.save_to_disk('_out/%08d' % data.frame)

    def add_radar(self, h_fov=15, v_fov=20, points_per_second=1500, range=50):
        weak_self = weakref.ref(self)
        radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(h_fov))
        radar_bp.set_attribute('vertical_fov', str(v_fov))
        radar_bp.set_attribute('points_per_second', str(points_per_second))
        radar_bp.set_attribute('range', str(range))
        radar_transform = carla.Transform(carla.Location(x=2.8, z=1.0))
        self.radar = self.world.spawn_actor(
            radar_bp, radar_transform, attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.radar.listen(lambda data: self.radar_callback(weak_self, data))

    def add_obstacle_sensor(self):
        weak_self = weakref.ref(self)
        obstacle_bp = self.world.get_blueprint_library().find('sensor.other.obstacle')
        obstacle_bp.set_attribute('distance', '50')
        obstacle_bp.set_attribute("only_dynamics", str(False))
        obstacle_bp.debug_linetrace = True
        obstacle_transform = carla.Transform(carla.Location(x=3, z=1))
        self.obstacle_sensor = self.world.spawn_actor(
            obstacle_bp, obstacle_transform, attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.obstacle_sensor.listen(
            lambda event: self.obstacle_callback(weak_self, event))

    @staticmethod
    def obstacle_callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        obs_loc = data.other_actor.get_location()
        obs_speed = compute_3D21d(data.other_actor.get_velocity())
        self.obstacle = Obstacle(obs_loc,
                                 data.distance, obs_speed)

    @staticmethod
    def radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))
        print(radar_data)


class Obstacle:
    def __init__(self, location, distance, velocity):
        self.location = location
        self.distance = distance
        self.velocity = velocity

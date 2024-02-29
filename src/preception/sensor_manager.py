import carla
import numpy as np


class SensorManager:
    def __init__(self, world, vehicle, config):
        self.world = world
        self.vehicle = vehicle
        self.add_radar()
        self.add_obstacle_sensor()
        # self.add_camera()

    def add_camera(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

        self.camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.camera_callback)

    def camera_callback(self, data):
        data.save_to_disk('_out/%08d' % data.frame)

    def add_radar(self, h_fov=35, v_fov=20, points_per_second=1500, range=50):
        radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(h_fov))
        radar_bp.set_attribute('vertical_fov', str(v_fov))
        radar_bp.set_attribute('points_per_second', str(points_per_second))
        radar_bp.set_attribute('range', str(range))
        radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
        self.radar = self.world.spawn_actor(
            radar_bp, radar_transform, attach_to=self.vehicle)
        self.radar.listen(self.radar_callback)

    def add_obstacle_sensor(self):
        obstacle_bp = self.world.get_blueprint_library().find('sensor.other.obstacle')
        obstacle_bp.set_attribute('distance', '50')
        obstacle_bp.debug_linetrace = True
        obstacle_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.obstacle_sensor = self.world.spawn_actor(
            obstacle_bp, obstacle_transform, attach_to=self.vehicle)
        self.obstacle_sensor.listen(self.obstacle_callback)

    def obstacle_callback(self, data):
        for obs in data:
            self.world.debug.draw_point(
                obs, size=0.8, color=carla.Color(25, 230, 10), life_time=0.05
            )

    def radar_callback(self, radar_data):
        velocity_range = 7.5
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))
        for detection in points:
            location = carla.Location(
                x=detection[0],
                y=detection[1],
                z=detection[2]
            )
            velocity = carla.Location(
                x=detection[0] + velocity_range * detection[3] / 127.0,
                y=detection[1] + velocity_range * detection[3] / 127.0,
                z=detection[2]
            )
            self.world.debug.draw_line(
                location, velocity, thickness=0.05, color=carla.Color(0, 255, 0), life_time=0.05, persistent_lines=False
            )
            self.world.debug.draw_point(
                location, size=0.1, color=carla.Color(0, 255, 0), life_time=0.05
            )

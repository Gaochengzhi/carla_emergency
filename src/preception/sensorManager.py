import carla
import numpy as np


class SensorManager:
    def __init__(self, world, vehicle, config):
        self.world = world
        self.vehicle = vehicle
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

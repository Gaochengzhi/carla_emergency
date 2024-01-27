import pygame
import time
import numpy as np
from agent.baseAgent import BaseAgent
import carla
from util import waypoints_center, get_ego_vehicle, connect_to_server
from view.color import WHITE, RED, GREEN, BLUE, BLACK, YELLOW
import os


class PyGameAgent(BaseAgent):
    def __init__(self, urban_waypoints, config):
        BaseAgent.__init__(self, "PyGame", config,
                           config["PortParameters"]["pygame_port"])
        self.config = config
        self.urban_waypoints = urban_waypoints
        # pyGame Instence
        self.pygame_frame = None
        self.camera = None
        self.width = config["PygameParameters"]["screen_width"]
        self.height = config["PygameParameters"]["screen_height"]
        self.vehicle = None
        self.screen = None
        self.font = None
        self.clock = None
        self.text_surface = None

    def run(self):
        client, world = connect_to_server(self.config)
        self.init_game_veiew(world, self.urban_waypoints, self.config)
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.Font(None, 20)
        self.clock = pygame.time.Clock()
        self.text_surface = self.font.render(
            'Started Emergency Simualtion', True, BLACK)  # White text

        while True:
            self.run_step(world)

    def init_game_veiew(self, world, urban_waypoints, config):
        pygame.init()
        while self.vehicle is None:
            self.vehicle = get_ego_vehicle(world)
            time.sleep(0.1)
            # world.wait_for_tick()
        self.init_game_camera(world, urban_waypoints, self.vehicle)

    def init_game_camera(self, world, urban_waypoints, vehicle):
        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute(
            'image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')
        # camera_location = waypoints_center(urban_waypoints)
        camera_location = carla.Location(x=0, y=0, z=0)
        self.camera = world.spawn_actor(
            camera_bp,
            carla.Transform(
                camera_location +
                carla.Location(z=100),
                carla.Rotation(pitch=-90),
            ),
            attach_to=vehicle,
        )
        self.camera.listen(self.update_pygame_frame)

    def image2pygame_surface(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4))  # RGBA format
        array = array[:, :, :3]  # Ignore alpha for RGB
        array = array[:, :, ::-1]  # Convert from BGR to RGB
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        return surface

    def update_pygame_frame(self, image):
        self.pygame_frame = self.image2pygame_surface(image)

    def run_step(self, world):
        dt = self.clock.tick()
        frame_rate = self.clock.get_fps()
        world.wait_for_tick()
        if self.pygame_frame is not None:
            self.screen.blit(self.pygame_frame, (0, 0))
            self.screen.blit(self.text_surface, (10, 10))
            fps_text = self.font.render(
                f'FPS: {frame_rate:.2f}', True, RED)
            self.screen.blit(fps_text, (10, 40))
            pygame.display.flip()

    def close(self):
        pygame.quit()
        self.camera.destroy()

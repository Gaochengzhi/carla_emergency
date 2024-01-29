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
        self.width_main = config["PygameParameters"]["screen_main_width"]
        self.height = config["PygameParameters"]["screen_height"]
        self.height_main = config["PygameParameters"]["screen_main_height"]
        self.vehicle = None
        self.screen = None
        self.font = None
        self.clock = None
        self.text_surface = None

        self.pygame_frame_x = (self.width - self.width_main) // 2
        self.pygame_frame_y = (self.height - self.height_main) // 2
        self.pygame_frame_rect = pygame.Rect(
            self.pygame_frame_x, self.pygame_frame_y, self.width_main, self.height_main)

        self.graph_rect = pygame.Rect(10, 10, 200, 100)
        self.graph_color = (0, 255, 0)
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        self.left_rect1 = pygame.Rect(0, 0, self.pygame_frame_x, self.center_y)
        self.left_rect2 = pygame.Rect(
            0, self.center_y, self.pygame_frame_x, self.center_y)
        self.right_rect1 = pygame.Rect(
            self.center_x + self.width_main // 2, 0, self.pygame_frame_x, self.center_y)
        self.right_rect2 = pygame.Rect(
            self.center_x + self.width_main // 2, self.center_y, self.pygame_frame_x, self.center_y)

        self.fps_history = []

    def run(self):
        client, world = connect_to_server(self.config)
        self.init_game_veiew(world, self.urban_waypoints, self.config)
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.Font(None, 20)
        self.clock = pygame.time.Clock()
        self.text_surface = self.font.render(
            'Started Emergency Simualtion', True, BLACK)  # White text
        pygame.draw.rect(self.screen, (0, 255, 0), self.right_rect1)
        pygame.draw.rect(self.screen, (0, 0, 255), self.left_rect2)
        pygame.draw.rect(self.screen, (255, 255, 0), self.right_rect2)

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
            'image_size_x', str(self.width_main))
        camera_bp.set_attribute('image_size_y', str(self.height_main))
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

    def draw_fps_line_chart(self):
        if len(self.fps_history) > 1:
            # points = [(i + self.center_x, int(self.center_y - fps)) for i, fps in enumerate(self.fps_history)]
            scaled_points = [(i + 10, int(self.center_y - fps * 10) - 10)
                             for i, fps in enumerate(self.fps_history)]
            pygame.draw.line(self.screen, (0, 0, 0),
                             (10, self.center_y - 10), (10, 0), 2)
            pygame.draw.line(self.screen, (0, 0, 0), (10, self.center_y - 10),
                             (self.pygame_frame_x, self.center_y - 10), 2)
            # pygame.draw.lines(self.screen, (0, 255, 0), False, points)
            pygame.draw.lines(self.screen, (0, 255, 0), False, scaled_points)

    def run_step(self, world):
        dt = self.clock.tick()
        frame_rate = self.clock.get_fps()
        self.fps_history.append(frame_rate)
        if len(self.fps_history) > 270:
            self.fps_history = self.fps_history[-270:]
        world.wait_for_tick()
        if self.pygame_frame is not None:
            self.screen.fill((255, 255, 255))
            self.screen.blit(self.pygame_frame, self.pygame_frame_rect.topleft)
            self.draw_fps_line_chart()
            # pygame.draw.rect(self.screen, (255, 0, 0), self.left_rect1)

            fps_text = self.font.render(
                f'frame_rate:{frame_rate}', True, (255, 0, 0))
            self.screen.blit(fps_text, (self.center_x, 40))

            pygame.display.flip()

    def close(self):
        pygame.quit()
        self.camera.destroy()

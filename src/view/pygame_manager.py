import pygame
import time
import numpy as np
import collections
import carla
from agent.baseAgent import BaseAgent
from util import waypoints_center, get_ego_vehicle, connect_to_server, batch_process_vehicles
from view.color import WHITE, RED, GREEN, BLUE, BLACK, YELLOW, PURPLE

class PyGameAgent(BaseAgent):
    def __init__(self, urban_waypoints, config):
        super().__init__("PyGame", config, config["PortParameters"]["pygame_port"])
        self.config = config
        self.urban_waypoints = urban_waypoints
        self.init_pygame_parameters(config)
        self.init_data_histories()

    def init_pygame_parameters(self, config):
        self.width = config["PygameParameters"]["screen_width"]
        self.height = config["PygameParameters"]["screen_height"]
        self.screen_main_width = config["PygameParameters"]["screen_main_width"]
        self.screen_main_height = config["PygameParameters"]["screen_main_height"]
        self.pygame_frame_rect = pygame.Rect((self.width - self.screen_main_width) // 2, 
                                             (self.height - self.screen_main_height) // 2, 
                                             self.screen_main_width, 
                                             self.screen_main_height)
        self.left_rects = [pygame.Rect(0, i * self.height // 4, self.screen_main_width // 4, self.height // 4) 
                           for i in range(4)]
        self.graph_colors = [RED, GREEN, BLUE, PURPLE]

    def init_data_histories(self):
        self.pygame_frame = []
        self.fps_history = collections.deque(maxlen=100)
        self.acceleration_history = collections.deque(maxlen=100)
        self.vertical_acceleration_history = collections.deque(maxlen=100)
        self.yaw_rate_history = collections.deque(maxlen=100)
        self.lateral_roll_angle_history = collections.deque(maxlen=100)

    def init_game_view(self, world, urban_waypoints, config):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.Font(None, 20)
        self.clock = pygame.time.Clock()
        self.vehicle = get_ego_vehicle(world)
        while self.vehicle is None:
            self.vehicle = get_ego_vehicle(world)
            time.sleep(0.1)

        self.init_game_camera(world, urban_waypoints, self.vehicle)

    def init_game_camera(self, world, urban_waypoints, vehicle):
        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute('image_size_x', str(self.screen_main_width))
        camera_bp.set_attribute('image_size_y', str(self.screen_main_height))
        camera_bp.set_attribute('fov', '90')
        camera_location = carla.Location(x=0, y=0, z=0)
        self.camera = world.spawn_actor(
            camera_bp,
            carla.Transform(camera_location + carla.Location(z=100), carla.Rotation(pitch=-90)),
            attach_to=vehicle,
        )
        self.camera.listen(self.update_pygame_frame)

    def image2pygame_surface(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))  # RGBA format
        array = array[:, :, :3]  # Ignore alpha for RGB
        array = array[:, :, ::-1]  # Convert from BGR to RGB
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        return surface

    def update_pygame_frame(self, image):
        self.pygame_frame = self.image2pygame_surface(image)

    def draw_data_curve(self, data_history, color, rect):
        if len(data_history) > 1:
            max_value = max(data_history)
            min_value = min(data_history)
            range_value = max_value - min_value if max_value - min_value else 1
            scaled_points = [(rect.x + i * rect.width / (len(data_history) - 1),
                              rect.y + rect.height - (data - min_value) / range_value * rect.height)
                             for i, data in enumerate(data_history)]
            pygame.draw.aalines(self.screen, color, False, scaled_points)

    def run_step(self, world):
        dt = self.clock.tick()
        frame_rate = self.clock.get_fps()
        self.fps_history.append(frame_rate)
        if len(self.fps_history) > 100:
            self.fps_history.popleft()
        if self.pygame_frame:
            self.screen.fill(WHITE)
            self.screen.blit(self.pygame_frame, self.pygame_frame_rect.topleft)

        # Draw curves for each data type
        data_histories = [self.acceleration_history, self.vertical_acceleration_history, 
                          self.yaw_rate_history, self.lateral_roll_angle_history]
        for i, history in enumerate(data_histories):
            self.draw_data_curve(history, self.graph_colors[i], self.left_rects[i])

        ego_vehicle = get_ego_vehicle(world)
        self.get_vehicle_data(ego_vehicle)
        batch_process_vehicles(world, ego_vehicle, 200, [-100, 100], self.change_lane)

        fps_text = self.font.render(f'Frame Rate: {frame_rate:.2f} FPS', True, BLACK)
        self.screen.blit(fps_text, (10, 10))
        pygame.display.flip()

    def get_vehicle_data(self, ego_vehicle):
        acceleration = ego_vehicle.get_acceleration()
        angular_velocity = ego_vehicle.get_angular_velocity()
        transform = ego_vehicle.get_transform()

        self.acceleration_history.append(acceleration.length())
        self.vertical_acceleration_history.append(acceleration.x)
        self.yaw_rate_history.append(angular_velocity.z)
        self.lateral_roll_angle_history.append(transform.rotation.roll)

    def change_lane(self, world, vehicle, ego):
        pass

    def run(self):
        client, world = connect_to_server(self.config)
        self.init_game_view(world, self.urban_waypoints, self.config)
        while True:
            self.run_step(world)

    def close(self):
        pygame.quit()
        if self.camera:
            self.camera.destroy()

# The rest of your original code...

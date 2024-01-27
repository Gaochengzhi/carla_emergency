import carla
import time
import matplotlib.pyplot as plt
from queue import Queue


class SensorManager:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.collision_sensor = None
        self.obstacle_sensor = None
        self.setup_sensors()
        self.count = 0
        # plt.ion()
        # set a plot page
        self.fig = plt.figure()
        self.data_queue = Queue()
        # self.axs = self.fig.add_subplot()

    def setup_sensors(self):
        blueprint_library = self.world.get_blueprint_library()
        # Create collision sensor
        # collision_bp = blueprint_library.find("sensor.other.collision")
        # self.collision_sensor = self.world.spawn_actor(
        #     collision_bp,
        #     carla.Transform(carla.Location(x=2.5, z=0.7)),  # Adjust position as needed
        #     attach_to=self.vehicle,
        #     attachment_type=carla.AttachmentType.Rigid,
        # )
        # # set attribute of sensor

        # # Create obstacle sensor
        # obstacle_bp = blueprint_library.find("sensor.other.obstacle")
        # obstacle_bp.set_attribute("distance", "20")
        # obstacle_bp.set_attribute("only_dynamics", "True")
        # obstacle_bp.set_attribute("debug_linetrace", "True")
        # obstacle_bp.set_attribute("hit_radius", "9")
        # self.world.tick()
        # self.obstacle_sensor = self.world.spawn_actor(
        #     obstacle_bp,
        #     carla.Transform(carla.Location(x=3.5, z=1.7)),  # Adjust position as needed
        #     attach_to=self.vehicle,
        #     attachment_type=carla.AttachmentType.Rigid,
        # )
        # # self.obstacle_sensor.listen(self.process_sensor_data)
        self.world.tick()

        # set attribute of sensor

    def destroy_sensors(self):
        # Destroy sensors
        if self.collision_sensor:
            self.collision_sensor.destroy()
        if self.obstacle_sensor:
            self.obstacle_sensor.destroy()

    # def process_sensor_data(self, data):
    #     if data.other_actor:
    #         vehicle_id = data.other_actor.id
    #         location_actor = data.other_actor.get_location()
    #         self.data_queue.put((location_actor.x, location_actor.y))

    #         print("current time:", time.time())
    #         print("this is:", data.other_actor.type_id)

    # def update_plot(self):
    #     while not self.data_queue.empty():
    #         x, y = self.data_queue.get()
    #         plt.scatter(x, y)
    #         plt.draw()
    #     if self.count % 8 == 0:
    #         plt.pause(0.01)
    #     if self.count % 100 == 0:
    #         plt.clf()

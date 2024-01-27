import carla
import random
import time
client = carla.Client("localhost", 2000)
client.set_timeout(3.0)
world = client.load_world("Town02")
map = world.get_map()
spectator = world.get_spectator()
loc = carla.Location(20, 40, 200.0)
rot = carla.Rotation(pitch=-90, yaw=0.0, roll=0.0)
spectator.set_transform(carla.Transform(loc, rot))
while True:
    time.sleep(2)

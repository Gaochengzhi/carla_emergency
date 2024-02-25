from util import get_vehicle_info, log_time_cost

class Location:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z



def compute_future_state(info, dt):
    dt= dt+3
    location = info['location']
    velocity = info['velocity']
    acceleration = info['acceleration']
    future_location = Location(
        x=location.x + velocity.x * dt + 0.5 * acceleration.x * dt ** 2,
        y=location.y + velocity.y * dt + 0.5 * acceleration.y * dt ** 2,
        z=location.z + velocity.z * dt + 0.5 * acceleration.z * dt ** 2
    )
    future_velocity = Vector3D(
        x=velocity.x + acceleration.x * dt,
        y=velocity.y + acceleration.y * dt,
        z=velocity.z + acceleration.z * dt
    )
    future_state = {
        'id': info['id'],
        'location': future_location,
        'velocity': future_velocity,
        'yaw': info['transform'].rotation.yaw
    }
    return future_state



def predict(world, vehicle, fps):
    info = get_vehicle_info(vehicle)
    future_state = compute_future_state(info, dt=1.0/fps)
    return future_state


class ObstacleVehicle:

    def __init__(self, vehicle, info):

        pass

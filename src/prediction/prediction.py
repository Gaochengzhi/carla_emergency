import carla


def get_vehicle_info(vehicle):
    location = vehicle.get_location()  # Returns carla.Location
    velocity = vehicle.get_velocity()  # Returns carla.Vector3D (m/s)
    acceleration = vehicle.get_acceleration()  # Returns carla.Vector3D (m/s^2)
    control = vehicle.get_control()

    transform = vehicle.get_transform()  # Returns carla.Transform

    # Compile the information into a dictionary for easy access
    vehicle_info = {
        'location': location,
        'velocity': velocity,
        'acceleration': acceleration,
        'control': control,
        'transform': transform,
    }

    return vehicle_info


def compute_future_state(info, dt=1.0):
    # Extract the current state from the info dictionary
    location = info['location']
    velocity = info['velocity']
    acceleration = info['acceleration']

    # Compute future position
    future_location = carla.Location(
        x=location.x + velocity.x * dt + 0.5 * acceleration.x * dt ** 2,
        y=location.y + velocity.y * dt + 0.5 * acceleration.y * dt ** 2,
        z=location.z + velocity.z * dt + 0.5 * acceleration.z * dt ** 2
    )

    # Compute future velocity
    future_velocity = carla.Vector3D(
        x=velocity.x + acceleration.x * dt,
        y=velocity.y + acceleration.y * dt,
        z=velocity.z + acceleration.z * dt
    )

    # Compile the future state into a dictionary
    future_state = {
        'location': future_location,
        'velocity': future_velocity
    }

    return future_state


def predict(world, vehicle, fps):
    info = get_vehicle_info(vehicle)
    future_state = compute_future_state(info, dt=1.0/fps)
    return future_state


class ObstacleVehicle:

    def __init__(self, vehicle, info):

        pass

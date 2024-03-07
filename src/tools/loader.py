from agent.baseline_vehicle_agent import BaselineVehicleAgent
import random
import carla


def load_agents(config):
    for i, agent_info in enumerate(config["agents"]):
        agent_info["ignore_traffic_light"] = False
        agent_info["fps"] = config["fps"]
        BaselineVehicleAgent(agent_info).start()


def load_batch_agents(config):

    config["spwan_list"] = random.sample(range(0, 50), 25)
    config["target_list"] = random.sample(range(50, 100), 25)
    agent_info = {}
    for i, spawn_target in enumerate(zip(config["spwan_list"], config["target_list"])):
        agent_info["name"] = f"agent_{i}"
        agent_info["fps"] = config["fps"]
        agent_info["port"] = int(9985+i)
        agent_info["start_point"] = spawn_target[0]
        agent_info["end_point"] = spawn_target[1]
        agent_info["type"] = "vehicle.tesla.model3"
        agent_info["traffic_agent_port"] = config["traffic_agent_port"]
        agent_info["main_port"] = config["main_port"]
        agent_info["ignore_traffic_light"] = False
        BaselineVehicleAgent(agent_info).start()


def load_conventional_agents(world, tm, config):
    try:
        spawn_point = world.get_map().get_spawn_points()
        config["spwan_list"] = random.sample(range(0, 90), 70)
        # config["spwan_list"] = [65, 64, 63] + random.sample(range(64, 100), 27)
        # config["spwan_list"] = [66]
        config["target_list"] = random.sample(range(100, 300), 70)
        # config["target_list"] = [1, 2, 6, 5, 6, 7, 8]
        for i, spwan_target in enumerate(zip(config["spwan_list"], config["target_list"])):
            vehicle_bp = world.get_blueprint_library().filter(
                "vehicle.tesla.model3*")[0]
            vehicle_bp.set_attribute('role_name', f"agent_{i}")
            vehicle = world.spawn_actor(vehicle_bp, spawn_point[spwan_target[0]])
            # tm.ignore_lights_percentage(vehicle, 100)
            # vehicle.set_simulate_physics(False)
            # vehicle.set_target_velocity(carla.Vector3D(0, 0, 105))
            vehicle.set_autopilot(True, tm.get_port())
            tm.vehicle_percentage_speed_difference(vehicle, 45)
    except Exception as e:
        print(f"load_conventional_agents error:{e}")
        pass

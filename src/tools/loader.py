from agent.baseline_vehicle_agent import BaselineVehicleAgent
import random


def load_agents(config):
    for i, agent_info in enumerate(config["agents"]):
        agent_info["ignore_traffic_light"] = False
        agent_info["fps"] = config["fps"]
        BaselineVehicleAgent(agent_info).start()


def load_batch_agents(config):

    config["spwan_list"] = random.sample(range(0, 101), 30)
    config["target_list"] = random.sample(range(103, 220), 30)
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
    spawn_point = world.get_map().get_spawn_points()
    agent_info = {}
    # config["spwan_list"] = random.sample(range(0, 200), 100)
    # config["spwan_list"] = [65, 64, 63, 66]
    config["spwan_list"] = range(194, 199)
    # config["target_list"] = random.sample(range(100, 300), 100)
    config["target_list"] = [1, 2, 6, 5, 6, 7, 8]
    for i, spwan_target in enumerate(zip(config["spwan_list"], config["target_list"])):
        vehicle_bp = world.get_blueprint_library().filter(
            "vehicle.tesla.model3*")[0]
        vehicle_bp.set_attribute('role_name', f"agent_{i}")
        vehicle = world.spawn_actor(vehicle_bp, spawn_point[spwan_target[0]])
        tm.ignore_lights_percentage(vehicle, 100)
        vehicle.set_autopilot(True, tm.get_port())
        tm.vehicle_percentage_speed_difference(vehicle, 2)

from agent.ego_vehicle_agent import EgoVehicleAgent


def load_agents(config):
    for i , agent_info in enumerate(config["agents"]):
        EgoVehicleAgent(agent_info).start()

from agent.ego_vehicle_agent import EgoVehicleAgent
def load_agents(config):
    for agent_info in config["agents"]:
        print(agent_info["name "]+"started")
        EgoVehicleAgent(agent_info).start()
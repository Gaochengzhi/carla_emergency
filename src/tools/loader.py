from agent.ego_vehicle_agent import EgoVehicleAgent


def load_agents(config):
    for i in range(100):
        # print(agent_info["name"]+" started")
        EgoVehicleAgent().start()

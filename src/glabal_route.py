import sys
import networkx as nx
import carla


class GlobalPlanner(object):
    def __init__(self, world, client, map, start_point, end_point):
        self.world = world
        self.client = client
        self.map = map
        self.route = map.get_topology()
        self.network = self.create_weight_network(self.route)

    def trace_router(self, start, goal):
        """
        start and end point are all carla waypoints
        """
        start_position = (start.x, start.y, start.z)
        end_position = (goal.x, goal.y, goal.z)
        shortest_path = nx.shortest_path(
            self.G, source=start_position, target=end_position
        )
        carla_waypoints = []
        for coord in shortest_path:
            location = carla.Location(x=coord[0], y=coord[1], z=coord[2])
            waypoint = self.map.get_waypoint(
                location, project_to_road=True, lane_type=carla.LaneType.Driving
            )
            carla_waypoints.append(waypoint)

        return carla_waypoints

    def create_weight_network(self, route, weight_pairs={}):
        G = nx.Graph()
        for edge in route:
            wp1, wp2 = edge[0].transform.location, edge[1].transform.location
            wp1_key = (wp1.x, wp1.y, wp1.z)
            wp2_key = (wp2.x, wp2.y, wp2.z)
            # Add edges to the graph
            # TODO: Add weight to the edges
            G.add_edge(wp1_key, wp2_key)  # Add weight if needed
        return G

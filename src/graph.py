import math
from math import radians, sin, cos, sqrt, atan2

class Graph:
    def __init__(self):
        self.nodes = {}  # node_id -> (lat, lon)
        self.adj_list = {}  # node_id -> list of (neighbor_id, cost)
        self.edges = []  # (u, v, cost) cho Bellman-Ford

    def add_node(self, node_id, lat, lon):
        self.nodes[node_id] = (lat, lon)
        self.adj_list[node_id] = []

    def add_edge(self, u, v, cost):
        self.adj_list[u].append((v, cost))
        self.edges.append((u, v, cost))

    def neighbors(self, node):
        return [v for v, _ in self.adj_list.get(node, [])]

    def cost(self, u, v):
        for neighbor, cost in self.adj_list[u]:
            if neighbor == v:
                return cost
        return float('inf')  # nếu không tồn tại

    def heuristic(self, u, v):
        lat1, lon1 = self.nodes[u]
        lat2, lon2 = self.nodes[v]
        return self.haversine_distance(lat1, lon1, lat2, lon2)
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371e3  # Bán kính Trái Đất (m)
        phi1 = radians(lat1)
        phi2 = radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)

        a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c
    
    def find_nearest_node(self, lat, lon):
        nearest_node = None
        min_distance = float('inf')

        for node_id, (node_lat, node_lon) in self.nodes.items():
            distance = self.haversine_distance(lat, lon, node_lat, node_lon)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node_id

        return nearest_node
    
    def has_edge(self, u, v):
        return v in self.neighbors(u)
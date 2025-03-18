import math

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
        # Tính khoảng cách địa lý giữa 2 điểm
        lat1, lon1 = self.nodes[u]
        lat2, lon2 = self.nodes[v]
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)
import math
from math import radians, sin, cos, sqrt, atan2
from scipy.spatial import KDTree
from geopy.distance import geodesic

class Graph:
    def __init__(self):
        self.nodes = {}  # node_id -> (lat, lon)
        self.adj_list = {}  # node_id -> list of (neighbor_id, cost)
        self.edges = []  # (u, v, cost) cho Bellman-Ford
        
        self.obstacles=set() # tap hop cac node_id la vat can
        self.blocked_edges=set() # (u,v) la canh bi chan

        self._kd_tree = None  # KDTree for nearest neighbor search
        self._node_ids = []  # list of node ids for KDTree

    def add_node(self, node_id, lat, lon):
        self.nodes[node_id] = (lat, lon)
        self.adj_list[node_id] = []
        self.kd_tree = None  # Reset KDTree when adding a new node
        self._node_ids.append(node_id)

    #Ham them vat can
    def add_obstacle(self,node_id):
        self.obstacles.add(node_id)

    def add_edge(self, u, v, cost):
        if u in self.obstacles or v in self.obstacles:
            return # Khong them canh neu u va v la vat can
        self.adj_list[u].append((v, cost))
        self.edges.append((u, v, cost))

    def neighbors(self, node):
        return [v for v, _ in self.adj_list.get(node, []) if v not in self.obstacles and (node,v) not in self.blocked_edges]

    def cost(self, u, v):
        if v in self.obstacles or (u,v) in self.blocked_edges:
            return float('inf')
        for neighbor, cost in self.adj_list.get(u, []):
            if neighbor == v:
                return cost
        return float('inf')


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
    
    def find_nearest_node_within_radius(self, lat, lon, initial_radius=10, step=10, max_radius=1000):
        if not self._kd_tree:
            self._build_kdtree()

        radius = initial_radius

        while radius <= max_radius:
            approx_radius_deg = radius / 111320  # Đổi mét sang độ
            idxs = self._kd_tree.query_ball_point([lat, lon], r=approx_radius_deg)

            if idxs:
                candidates = []
                for idx in idxs:
                    node_id = self._node_ids[idx]
                    if node_id in self.obstacles:
                        continue
                    node_lat, node_lon = self.nodes[node_id]
                    distance_m = geodesic((lat, lon), (node_lat, node_lon)).meters
                    candidates.append((distance_m, node_id))

                candidates.sort()
                return candidates[0][1]

            radius += step

        return None

    def _build_kdtree(self):
        self._node_ids = list(self.nodes.keys())
        coords = [self.nodes[node_id] for node_id in self._node_ids]
        self._kd_tree = KDTree(coords)


#------------Bo sung them----------------

    def remove_obstacle(self, node_id):
        self.obstacles.discard(node_id)     # Xoa vat can

    def is_obstacle(self,node_id):
        return node_id in self.obstacles    # kiem tra co phai vat can hay khong 

    def block_edge(self, u,v):
        self.blocked_edges.add((u,v))         # Chan canh tu u den v - duong cam

    def unblock_edge(self, u,v):
        self.blocked_edges.discard((u,v))       # mo lai canh u->v

    def is_blocked_edge(self, u,v):
        return (u,v) in self.blocked_edges      # Kiem tra canh co bi chan khong 
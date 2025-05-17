import math
from math import radians, sin, cos, sqrt, atan2
from scipy.spatial import KDTree
from geopy.distance import geodesic

class Graph:
    def __init__(self):
        self.nodes = {}  # node_id -> (lat, lon)
        self.adj_list = {}  # node_id -> list of (neighbor_id, cost)
        self.edges = []  # (u, v, cost) cho Bellman-Ford
        self.node_coords = {} # Dictionary để lưu tọa độ các node (nút)
        self.obstacles=set() # tap hop cac node_id la vat can
        self._removed_edges={}  # node_id->list of(u,v,cost)

        self._kd_tree = None  # KDTree for nearest neighbor search
        self._node_ids = []  # list of node ids for KDTree

    def add_node(self, node_id, lat, lon):
        
        self.nodes[node_id] = (lat, lon)
        self.adj_list[node_id] = []
        self.kd_tree = None  # Reset KDTree when adding a new node
        self._node_ids.append(node_id)
        self.node_coords[node_id] = (lat, lon)

    def add_edge(self, u, v, cost):
        if u in self.obstacles or v in self.obstacles:
            return # Khong them canh neu u va v la vat can
        self.adj_list[u].append((v, cost))
        self.edges.append((u, v, cost))

    def neighbors(self, node):
        return [v for v, _ in self.adj_list.get(node, []) if v not in self.obstacles ]

    def cost(self, u, v):
        if v in self.obstacles:
            return float('inf')
        for neighbor, cost in self.adj_list.get(u, []):
            if neighbor == v:
                return cost
        return float('inf')
    
    def has_edge(self, u, v):
        return v in self.neighbors(u)
    
    def heuristic(self, u, v):
        # Sử dụng hàm heuristic1 làm mặc định
        return self.heuristic1(u, v)

    def heuristic1(self, u, v):
        return geodesic(self.nodes[u], self.nodes[v]).meters

    def heuristic2(self, u, v):
        lat1, lon1 = self.nodes[u]
        lat2, lon2 = self.nodes[v]
        lat_mean = radians((lat1 + lat2) / 2)
        dx = (lon2 - lon1) * 111320 * cos(lat_mean)
        dy = (lat2 - lat1) * 111320
        return sqrt(dx**2 + dy**2)
    
    def heuristic3(self, u, v):
        # Ưu tiên đi theo góc lệch so với thẳng tắp từ u đến v
        lat1, lon1 = self.nodes[u]
        lat2, lon2 = self.nodes[v]
        lat_mean = radians((lat1 + lat2) / 2)
        dx = (lon2 - lon1) * 111320 * cos(lat_mean)
        dy = (lat2 - lat1) * 111320
        angle = atan2(dy, dx)
        multiplier = min(1 + abs(angle) / (math.pi / 4), 2)  # Giới hạn tối đa gấp đôi
        return sqrt(dx**2 + dy**2) * multiplier

    def find_nearest_node(self, lat, lon):
        nearest_node = None
        min_distance = float('inf')

        for node_id, _ in self.nodes.items():
            distance = self.heuristic(node_id, (lat, lon))
            if distance < min_distance:
                min_distance = distance
                nearest_node = node_id

        return nearest_node
    
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

                if not candidates:
                    return None
                candidates.sort()
                return candidates[0][1]
            radius += step
        return None

    def _build_kdtree(self):
        self._node_ids = list(self.nodes.keys())
        coords = [self.nodes[node_id] for node_id in self._node_ids]
        self._kd_tree = KDTree(coords)


#------------Bo sung them----------------

    #Ham them vat can
    def add_obstacle(self,node_id):
        """Đánh dấu node là vật cản và loại bỏ các cạnh liên quan"""
        if node_id not in self.nodes:
            raise ValueError(f"Node {node_id} không tồn tại.")
        self.obstacles.add(node_id)
        removed=[] # danh sach cac canh bi xoa
        #Xóa các cạnh đi từ node này
        for v, cost in self.adj_list.get(node_id,[]):
            removed.append((node_id,v,cost))
        if node_id in self.adj_list:
            self.adj_list[node_id]=[]
        #Xoa cac canh den node_id
        for u in self.adj_list:
            new_neighbors=[]
            for v,cost in self.adj_list[u]:
                if v==node_id:
                    removed.append((u,v,cost))
                else:
                    new_neighbors.append((v,cost))
            self.adj_list[u]=new_neighbors
        #Xoa cac canh lien quan trong self.edges
        self.edges=[(u,v,c) for (u,v,c) in self.edges if u!=node_id and v!=node_id]
        #Luu lai cac canh da bi loai bo
        self._removed_edges[node_id]=removed
        self._kd_tree=None  # reset neu dung KDTree

    def add_obstacles(self, node_list):
        for node_id in node_list:
            self.add_obstacle(node_id)

    def remove_obstacle(self, node_id):
        """Go node khoi danh sach va khoi phuc lai cac canh da xoa """
        self.obstacles.discard(node_id)     # Xoa vat can
        if node_id in self._removed_edges:
            for u,v,cost in self._removed_edges[node_id]:
                if u in self.nodes and v in self.nodes:
                    if not any(v2 == v for v2,_ in self.adj_list[u]):
                        self.adj_list[u].append((v,cost))
                        self.edges.append((u,v,cost))
            del self._removed_edges[node_id]
        self._kd_tree= None

    def remove_obstacles(self,node_list):
        for node_id in node_list:
            self.remove_obstacle(node_id)

    def is_obstacle(self,node_id):
        return node_id in self.obstacles    # kiem tra co phai vat can hay khong 

  
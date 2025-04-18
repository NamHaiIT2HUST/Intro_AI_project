import math
import threading
from math import radians, sin, cos, sqrt, atan2
from scipy.spatial import KDTree
from geopy.distance import geodesic

class Graph:
    def __init__(self):
        self.nodes = {}  # node_id -> (lat, lon)
        self.adj_list = {}  # node_id -> list of (neighbor_id, cost)
        self.edges = []  # (u, v, cost) cho Bellman-Ford

        self._kd_tree = None  # KDTree for nearest neighbor search
        self._node_ids = []  # list of node ids for KDTree

    def add_node(self, node_id, lat, lon):
        """Thêm một node vào đồ thị với id, lat, lon"""
        self.nodes[node_id] = (lat, lon)
        self.adj_list[node_id] = []
        self.kd_tree = None  # Reset KDTree when adding a new node
        self._node_ids.append(node_id)

    def add_edge(self, u, v, cost):
        """Thêm một cạnh vào đồ thị với id của 2 node và cost"""
        self.adj_list[u].append((v, cost))
        self.edges.append((u, v, cost))

    def neighbors(self, node):
        """Trả về danh sách các node kề với node cho trước"""
        return [v for v, _ in self.adj_list.get(node, [])]

    def cost(self, u, v):
        """Trả về cost của cạnh từ u đến v"""
        for neighbor, cost in self.adj_list.get(u, []):
            if neighbor == v:
                return cost
        return float('inf')

    def heuristic(self, u, v):
        """Trả về heuristic giữa 2 node u và v"""
        return geodesic(self.nodes[u], self.nodes[v]).meters
    
    def find_nearest_node(self, lat, lon):
        """Tìm node gần nhất với tọa độ lat, lon"""
        nearest_node = None
        min_distance = float('inf')

        for node_id, (node_lat, node_lon) in self.nodes.items():
            distance = self.haversine_distance(lat, lon, node_lat, node_lon)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node_id

        return nearest_node
    
    def has_edge(self, u, v):
        """Kiểm tra xem có cạnh từ u đến v hay không"""
        return v in self.neighbors(u)
    
    def find_nearest_node_within_radius(self, lat, lon, initial_radius=10, step=10, max_radius=1000):
        """
        Tìm node gần nhất trong bán kính cho trước.
        Nếu không tìm thấy, tăng bán kính lên theo bước cho đến khi đạt max_radius."""
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

    def is_point_near_segment(self, lat, lon, lat1, lon1, lat2, lon2, threshold=5):
        """
        Kiểm tra xem điểm (lat, lon) 
        có gần đoạn thẳng từ (lat1, lon1) đến (lat2, lon2) trong bán kính threshold hay không."""
        x, y = radians(lon), radians(lat)
        x1, y1 = radians(lon1), radians(lat1)
        x2, y2 = radians(lon2), radians(lat2)

        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return geodesic((lat, lon), (lat1, lon1)).meters <= threshold
        
        t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
        t = max(0, min(1, t))
        nearest_x = x1 + t * dx
        nearest_y = y1 + t * dy

        nearest_lat, nearest_lon = math.degrees(nearest_y), math.degrees(nearest_x)
        distance = geodesic((lat, lon), (nearest_lat, nearest_lon)).meters
        return distance <= threshold
    
    def find_edges_near_point_async(self, lat, lon, radius=3, callback=None):
        """
        Tìm các cạnh gần điểm (lat, lon) trong bán kính cho trước."""
        def worker():
            results = []
            radius_deg = radius / 111320  # đổi m sang độ
            lat_min = lat - radius_deg
            lat_max = lat + radius_deg
            lon_min = lon - radius_deg
            lon_max = lon + radius_deg

            for u, v, _ in self.edges:
                lat1, lon1 = self.nodes[u]
                lat2, lon2 = self.nodes[v]

                # Nếu đoạn không nằm trong vùng bbox → bỏ qua
                if not (
                    min(lat1, lat2) <= lat_max and max(lat1, lat2) >= lat_min and
                    min(lon1, lon2) <= lon_max and max(lon1, lon2) >= lon_min
                ):
                    continue

                if self.is_point_near_segment(lat, lon, lat1, lon1, lat2, lon2, threshold=radius):
                    results.append((u, v))

            if callback:
                callback(results)

        import threading
        threading.Thread(target=worker, daemon=True).start()

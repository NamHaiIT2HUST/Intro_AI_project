from abc import ABC, abstractmethod
from queue import PriorityQueue, Queue
import threading
import time

class Algorithm(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def run(self, start, goal, graph):
        pass

    def reconstruct_path(self, start, goal, came_from):
        path = []
        current = goal
        while current != start:
            path.append(current)
            if came_from.get(current) is None:
                return None
            current = came_from[current]
        path.append(start)
        return path[::-1]

class AStar(Algorithm):
    def __init__(self, heuristic, graph = None):
        super().__init__()
        self.graph = graph
        self.heuristic = heuristic

    def run(self, start, goal, graph):
        count_node = 0
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        f_score = {node: float('inf') for node in graph.nodes}
        f_score[start] = graph.heuristic(start, goal)
        g_score = {}
        g_score[start] = 0

        while not open_set.empty():
            _, current = open_set.get()
            count_node += 1
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + graph.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
        return count_node, None

class BidirectionalAStar(Algorithm):
    def __init__(self, heuristic, graph=None):
        super().__init__()
        self.heuristic = heuristic
        self.graph = graph
        
    def run(self, start, goal, graph):
        # Trường hợp đặc biệt
        if start == goal:
            return 0, [start]
            
        # Khởi tạo các priority queue với giá trị f
        open_start = PriorityQueue()
        open_goal = PriorityQueue()
        
        # Đếm số node được mở
        count_node = 0
        
        # Các set theo dõi node đã được đưa vào queue
        open_set_start = {start}
        open_set_goal = {goal}
        
        # Các set theo dõi node đã xử lý
        closed_start = set()
        closed_goal = set()
        
        # Các map lưu đường đi
        came_from_start = {}
        came_from_goal = {}
        
        # G-scores
        g_start = {start: 0}
        g_goal = {goal: 0}
        
        # F-scores
        f_start = {start: self.heuristic(start, goal)}
        f_goal = {goal: self.heuristic(goal, start)}
        
        # Đưa các node khởi đầu vào queue
        open_start.put((f_start[start], start))
        open_goal.put((f_goal[goal], goal))
        
        # Biến lưu kết quả tốt nhất
        best_node = None
        best_path_cost = float('inf')
        
        # Điều kiện μ cho consistent bound
        mu = float('inf')
        
        # Chọn hướng tìm kiếm dựa trên kích thước của các hàng đợi
        while not open_start.empty() and not open_goal.empty():
            # Kiểm tra điều kiện dừng: nếu μ nhỏ hơn f_min của cả hai hướng
            f_min_start = open_start.queue[0][0] if not open_start.empty() else float('inf')
            f_min_goal = open_goal.queue[0][0] if not open_goal.empty() else float('inf')
            
            if mu <= f_min_start + f_min_goal:
                break
                
            # Quyết định hướng tiếp theo dựa trên kích thước của hàng đợi
            # Ưu tiên hướng có ít nút trong hàng đợi hơn để cân bằng tìm kiếm
            if len(open_set_start) <= len(open_set_goal):
                count_node += self._expand_forward(open_start, open_set_start, closed_start, 
                                              g_start, f_start, came_from_start,
                                              open_set_goal, closed_goal, g_goal,
                                              start, goal, graph, best_path_cost, mu)
            else:
                count_node += self._expand_backward(open_goal, open_set_goal, closed_goal, 
                                               g_goal, f_goal, came_from_goal,
                                               open_set_start, closed_start, g_start,
                                               start, goal, graph, best_path_cost, mu)
            
            # Cập nhật best_path_cost và best_node sau mỗi lần mở rộng
            for node in open_set_start & open_set_goal:
                if node in g_start and node in g_goal:
                    path_cost = g_start[node] + g_goal[node]
                    if path_cost < best_path_cost:
                        best_path_cost = path_cost
                        best_node = node
                        mu = path_cost
        
        # Tạo đường đi kết quả
        if best_node:
            path_start = self.reconstruct_path(start, best_node, came_from_start)
            path_goal = self.reconstruct_path(goal, best_node, came_from_goal)[::-1]
            
            # Loại bỏ nút trùng
            if len(path_goal) > 0:
                path_goal = path_goal[1:]
                
            return count_node, path_start + path_goal
        else:
            print("No path found")
            return count_node, None
            
    def _expand_forward(self, open_queue, open_set, closed, g_score, f_score, came_from,
                       other_open_set, other_closed, other_g_score,
                       start, goal, graph, best_cost, mu):
        """Mở rộng tìm kiếm từ phía start."""
        if open_queue.empty():
            return 0
            
        # Lấy node có f nhỏ nhất
        _, current = open_queue.get()
        open_set.remove(current)
        
        # Nếu node đã xử lý rồi, bỏ qua
        if current in closed:
            return 1
            
        # Thêm vào tập đã xử lý
        closed.add(current)
        
        # Nếu đã tìm thấy đường đi tốt hơn rồi, bỏ qua
        if current in other_g_score and g_score[current] + other_g_score[current] >= mu:
            return 1
            
        # Mở rộng các node kề
        for neighbor in graph.neighbors(current):
            # Bỏ qua nếu đã xử lý
            if neighbor in closed:
                continue
                
            # Tính g mới
            tentative_g = g_score[current] + graph.cost(current, neighbor)
            
            # Cập nhật nếu tốt hơn
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                
                # Kiểm tra điều kiện pruning
                if neighbor in other_g_score and tentative_g + other_g_score[neighbor] < mu:
                    mu = tentative_g + other_g_score[neighbor]
                
                # Thêm vào queue nếu chưa có
                if neighbor not in open_set:
                    open_queue.put((f_score[neighbor], neighbor))
                    open_set.add(neighbor)
                    
        return 1
        
    def _expand_backward(self, open_queue, open_set, closed, g_score, f_score, came_from,
                        other_open_set, other_closed, other_g_score,
                        start, goal, graph, best_cost, mu):
        """Mở rộng tìm kiếm từ phía goal."""
        if open_queue.empty():
            return 0
            
        # Lấy node có f nhỏ nhất
        _, current = open_queue.get()
        open_set.remove(current)
        
        # Nếu node đã xử lý rồi, bỏ qua
        if current in closed:
            return 1
            
        # Thêm vào tập đã xử lý
        closed.add(current)
        
        # Nếu đã tìm thấy đường đi tốt hơn rồi, bỏ qua
        if current in other_g_score and g_score[current] + other_g_score[current] >= mu:
            return 1
            
        # Mở rộng các node kề
        for neighbor in graph.neighbors(current):
            # Bỏ qua nếu đã xử lý
            if neighbor in closed:
                continue
                
            # Tính g mới
            tentative_g = g_score[current] + graph.cost(current, neighbor)
            
            # Cập nhật nếu tốt hơn
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + self.heuristic(neighbor, start)
                
                # Kiểm tra điều kiện pruning
                if neighbor in other_g_score and tentative_g + other_g_score[neighbor] < mu:
                    mu = tentative_g + other_g_score[neighbor]
                
                # Thêm vào queue nếu chưa có
                if neighbor not in open_set:
                    open_queue.put((f_score[neighbor], neighbor))
                    open_set.add(neighbor)
                    
        return 1

class Greedy(Algorithm):
    def __init__(self, heuristic, graph = None):
        super().__init__()
        self.graph = graph
        self.heuristic = heuristic

    def run(self, start, goal, graph):
        count_node = 0
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        while not open_set.empty():
            count_node += 1
            _, current = open_set.get()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    open_set.put((graph.heuristic(neighbor, goal), neighbor))
        return count_node, None

class Dijkstra(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        count_node = 0
        came_from = {}
        open_set = PriorityQueue()
        open_set.put((0, start))

        g_score = {node: float('inf') for node in graph.nodes}
        g_score[start] = 0

        while not open_set.empty():
            count_node += 1
            _, current = open_set.get()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    open_set.put((g_score[neighbor], neighbor))
        return count_node, None
    
class BFS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        count_node = 0
        came_from = {}
        open_set = Queue()
        open_set.put(start)
        came_from[start] = None

        while not open_set.empty():
            count_node += 1
            current = open_set.get()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    open_set.put(neighbor)
        return count_node, None
    
class DFS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        count_node = 0
        came_from = {}
        open_set = []
        open_set.append(start)
        came_from[start] = None

        while open_set:
            count_node += 1
            current = open_set.pop()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    open_set.append(neighbor)
        return count_node, None

class BellmanFord(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        count_node = 0
        dist = {node: float('inf') for node in graph.nodes}
        prev = {node: None for node in graph.nodes}
        dist[start] = 0

        for _ in range(len(graph.nodes) - 1):
            for edge in graph.edges:
                count_node += 1
                u, v, w = edge
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
            
        for edge in graph.edges:
            count_node += 1
            u, v, w = edge
            if dist[u] + w < dist[v]:
                return None
            
        return count_node, self.reconstruct_path(start, goal, prev)

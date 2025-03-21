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
    def __init__(self, heuristic, graph = None):
        super().__init__()
        self.heuristic = heuristic
        self.graph = graph
        self.lock = threading.Lock()
        self.found = threading.Event()
        self.meeting_point = None
        self.path_start = []
        self.path_goal = []

    def run(self, start, goal, graph):
        self.queue_start = PriorityQueue()
        self.queue_goal = PriorityQueue()

        count_node_1 = [0]
        count_node_2 = [0]
        
        self.came_from_start = {}
        self.came_from_goal = {}
        
        self.g_start = {start: 0}
        self.g_goal = {goal: 0}
        
        self.queue_start.put((0, start))
        self.queue_goal.put((0, goal))
        
        t1 = threading.Thread(target=self.expand, args=(count_node_1, goal, self.queue_start, self.came_from_start, self.g_start, self.g_goal, graph))
        t2 = threading.Thread(target=self.expand, args=(count_node_2, start, self.queue_goal, self.came_from_goal, self.g_goal, self.g_start, graph))
        
        t1.start()
        t2.start()
        
        t1.join()
        t2.join()
        
        if self.meeting_point:
            path_start = self.reconstruct_path(start, self.meeting_point, self.came_from_start)
            path_goal = self.reconstruct_path(goal, self.meeting_point, self.came_from_goal)[::-1]
            return count_node_1[0] + count_node_2[0], path_start + path_goal
        else:
            print("No path found")
        return count_node_1[0] + count_node_2[0], None
    
    def expand(self, count_node, goal, queue, came_from, g_score, other_g_score, graph):
       while not queue.empty() and not self.found.is_set():
            count_node[0] += 1
            _, curr = queue.get()
            
            if curr in other_g_score:
                with self.lock:
                    if not self.found.is_set():
                        self.found.set()
                        self.meeting_point = curr
                return
            
            for neighbor in graph.neighbors(curr):
                ten_g_score = g_score[curr] + graph.cost(curr, neighbor)
                if neighbor not in g_score or ten_g_score < g_score[neighbor]:
                    came_from[neighbor] = curr
                    g_score[neighbor] = ten_g_score
                    queue.put((ten_g_score + self.heuristic(neighbor, goal), neighbor))


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

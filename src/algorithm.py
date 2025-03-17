from abc import ABC, abstractmethod
from queue import PriorityQueue, Queue

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
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

class AStar(Algorithm):
    def __init__(self, heuristic, graph = None):
        super().__init__()
        self.graph = graph
        self.heuristic = heuristic

    def run(self, start, goal, graph):
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        f_score = {node: float('inf') for node in graph.nodes}
        f_score[start] = graph.heuristic(start, goal)
        g_score = {}
        g_score[start] = 0

        while not open_set.empty():
            _, current = open_set.get()
            if current == goal:
                return self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + graph.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
        return None
    

class Greedy(Algorithm):
    def __init__(self, heuristic, graph = None):
        super().__init__()
        self.graph = graph
        self.heuristic = heuristic

    def run(self, start, goal, graph):
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        while not open_set.empty():
            _, current = open_set.get()
            if current == goal:
                return self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    open_set.put((graph.heuristic(neighbor, goal), neighbor))
        return None

class Dijkstra(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        came_from = {}
        open_set = PriorityQueue()
        open_set.put((0, start))

        g_score = {node: float('inf') for node in graph.nodes}
        g_score[start] = 0

        while not open_set.empty():
            _, current = open_set.get()
            if current == goal:
                return self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    open_set.put((g_score[neighbor], neighbor))
        return None
    
class BFS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        came_from = {}
        open_set = Queue()
        open_set.put(start)
        came_from[start] = None

        while not open_set.empty():
            current = open_set.get()
            if current == goal:
                return self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    open_set.put(neighbor)
        return None
    
class DFS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        came_from = {}
        open_set = []
        open_set.append(start)
        came_from[start] = None

        while open_set:
            current = open_set.pop()
            if current == goal:
                return self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    open_set.append(neighbor)
        return None

class BellmanFord(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        dist = {node: float('inf') for node in graph.nodes}
        prev = {node: None for node in graph.nodes}
        dist[start] = 0

        for _ in range(len(graph.nodes) - 1):
            for edge in graph.edges:
                u, v, w = edge
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
            
        for edge in graph.edges:
            u, v, w = edge
            if dist[u] + w < dist[v]:
                return None
            
        return self.reconstruct_path(start, goal, prev)

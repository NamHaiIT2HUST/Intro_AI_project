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
        open_queue = PriorityQueue()
        open_queue.put((0, start))
        open_set = set()
        open_set.add(start)
        came_from = {}
        f_score = {node: float('inf') for node in graph.nodes}
        f_score[start] = self.heuristic(start, goal)
        g_score = {}
        g_score[start] = 0

        while not open_queue.empty():
            _, current = open_queue.get()
            count_node += 1
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor in graph.obstacles :
                    continue

                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_queue.put((f_score[neighbor], neighbor))
                        open_set.add(neighbor)
        return count_node, None

class BidirectionalAStar(Algorithm):
    def __init__(self, heuristic, graph=None):
        super().__init__()
        self.heuristic = heuristic
        self.graph = graph

    def run(self, start, goal, graph):
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None
        if start == goal:
            return 0, [start]

        open_start = PriorityQueue()
        open_goal = PriorityQueue()
        open_set_start = {start}
        open_set_goal = {goal}
        closed_start = set()
        closed_goal = set()

        came_from_start = {}
        came_from_goal = {}

        g_start = {start: 0}
        g_goal = {goal: 0}
        f_start = {start: self.heuristic(start, goal)}
        f_goal = {goal: self.heuristic(goal, start)}

        open_start.put((f_start[start], start))
        open_goal.put((f_goal[goal], goal))

        best_node = None
        best_path_cost = float('inf')
        mu = float('inf')
        count_node = 0

        while not open_start.empty() and not open_goal.empty():
            f_min_start = open_start.queue[0][0]
            f_min_goal = open_goal.queue[0][0]
            if mu <= f_min_start + f_min_goal:
                break

            if len(open_set_start) <= len(open_set_goal):
                count_node += self._expand(
                    open_start, open_set_start, closed_start, g_start, f_start, came_from_start,
                    open_set_goal, g_goal, goal, graph
                )
            else:
                count_node += self._expand(
                    open_goal, open_set_goal, closed_goal, g_goal, f_goal, came_from_goal,
                    open_set_start, g_start, start, graph
                )

            # Kiểm tra node giao nhau giữa 2 phía
            intersection = open_set_start & open_set_goal
            for node in intersection:
                if node in g_start and node in g_goal:
                    cost = g_start[node] + g_goal[node]
                    if cost < best_path_cost:
                        best_path_cost = cost
                        best_node = node
                        mu = cost

        if best_node:
            path_start = self.reconstruct_path(start, best_node, came_from_start)
            path_goal = self.reconstruct_path(goal, best_node, came_from_goal)[::-1]
            if path_goal and path_goal[0] == best_node:
                path_goal = path_goal[1:]
            return count_node, path_start + path_goal
        else:
            return count_node, None

    def _expand(self, open_queue, open_set, closed, g_score, f_score, came_from,
                other_open_set, other_g_score, target, graph):
        if open_queue.empty():
            return 0

        _, current = open_queue.get()
        if current in closed:
            return 0
        open_set.discard(current)
        closed.add(current)

        for neighbor in graph.neighbors(current):
            if neighbor in closed:
                continue
            tentative_g = g_score[current] + graph.cost(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + self.heuristic(neighbor, target)
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
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None # kiem tra dau vao va dich
        count_node = 0
        open_set = PriorityQueue()
        open_set.put((0, start))
        closed = set()
        closed.add(start)
        came_from = {}
        while not open_set.empty():
            count_node += 1
            _, current = open_set.get()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                # if neighbor in graph.obstacles:
                #     continue
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    if neighbor not in closed:
                        open_set.put((self.heuristic(neighbor, goal), neighbor))
                        closed.add(neighbor)
        return count_node, None

class Dijkstra(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None
        count_node = 0
        came_from = {}
        open_queue = PriorityQueue()
        open_queue.put((0, start))
        open_set = {start}

        g_score = {node: float('inf') for node in graph.nodes}
        g_score[start] = 0

        while not open_queue.empty():
            count_node += 1
            _, current = open_queue.get()
            open_set.remove(current)
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    if neighbor not in open_set:
                        open_queue.put((g_score[neighbor], neighbor))
                        open_set.add(neighbor)
        return count_node, None
    
class BFS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None
        count_node = 0
        came_from = {}
        open_set = Queue()
        open_set.put(start)
        came_from[start] = None
        closed = set()
        closed.add(start)

        while not open_set.empty():
            count_node += 1
            current = open_set.get()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor in closed:
                    continue
                closed.add(neighbor)
                came_from[neighbor] = current
                open_set.put(neighbor)
        return count_node, None
    
class DFS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None
        count_node = 0
        came_from = {}
        open_set = []
        open_set.append(start)
        came_from[start] = None
        closed = set()
        closed.add(start)

        while open_set:
            count_node += 1
            current = open_set.pop()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor in closed:
                    continue
                closed.add(neighbor)
                came_from[neighbor] = current
                open_set.append(neighbor)
        return count_node, None

class BellmanFord(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None
        count_node = 0
        dist = {node: float('inf') for node in graph.nodes}
        prev = {node: None for node in graph.nodes}
        dist[start] = 0

        for _ in range(len(graph.nodes) - 1):
            for edge in graph.edges:
                u, v, w = edge
                if u in graph.obstacles or v in graph.obstacles :
                    continue  # Bỏ qua cạnh này
                count_node += 1
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
            
        for edge in graph.edges:
            u, v, w = edge
            if u in graph.obstacles or v in graph.obstacles :
                continue  # Bỏ qua cạnh này
            count_node += 1
            if dist[u] + w < dist[v]:
                return None
            
        return count_node, self.reconstruct_path(start, goal, prev)

class UCS(Algorithm):
    def __init__(self, graph = None):
        super().__init__()
        self.graph = graph
    
    def run(self, start, goal, graph):
        if start in graph.obstacles or goal in graph.obstacles:
            return 0, None
        count_node = 0
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {node: float('inf') for node in graph.nodes}
        g_score[start] = 0

        while not open_set.empty():
            count_node += 1
            _, current = open_set.get()
            if current == goal:
                return count_node, self.reconstruct_path(start, goal, came_from)
            for neighbor in graph.neighbors(current):
                if neighbor in graph.obstacles:
                    continue
                tentative_g_score = g_score[current] + graph.cost(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    open_set.put((g_score[neighbor], neighbor))
        return count_node, None
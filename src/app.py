from tkintermapview import TkinterMapView
import customtkinter
from graph import Graph
from algorithm import AStar, Dijkstra, BFS, DFS, Greedy, BellmanFord
import math

class App(customtkinter.CTk):
    APP_NAME = "Map View - Bách Khoa"
    CENTER_LAT, CENTER_LON = 21.004981, 105.8443
    ALGORITHMS = {
        "A*": lambda self: AStar(self.distance),
        "Dijkstra": lambda _: Dijkstra(),
        "Greedy": lambda self: Greedy(self.distance),
        "BFS": lambda _: BFS(),
        "DFS": lambda _: DFS(),
        "Bellman-Ford": lambda _: BellmanFord()
    }

    def __init__(self):
        super().__init__()
        self.title(self.APP_NAME)
        self.geometry("900x600")
        self.resizable(False, False)

        self.graph = Graph()
        self.start_node = None
        self.goal_node = None
        self.markers = []
        self.path_line = None

        self._setup_ui()
        self._initialize_map()

    def _setup_ui(self):
        # Bản đồ
        self.map_widget = TkinterMapView(self, width=700, height=600, corner_radius=0)
        self.map_widget.pack(side="left", fill="both", expand=False)

        # Control Panel
        self.panel = customtkinter.CTkFrame(self, width=200)
        self.panel.pack(side="right", fill="y")

        self.alg_label = customtkinter.CTkLabel(self.panel, text="Thuật toán:")
        self.alg_label.pack(pady=(20, 5))

        self.alg_selector = customtkinter.CTkComboBox(
            self.panel, values=list(self.ALGORITHMS.keys())
        )
        self.alg_selector.pack(pady=10)

        self.run_button = customtkinter.CTkButton(self.panel, text="Tìm đường", command=self.run_algorithm)
        self.run_button.pack(pady=20)

        self.clear_button = customtkinter.CTkButton(self.panel, text="Xoá chọn", command=self.clear_selection)
        self.clear_button.pack(pady=10)

    def _initialize_map(self):
        self.map_widget.set_position(self.CENTER_LAT, self.CENTER_LON)
        self.map_widget.set_zoom(17)
        self.map_widget.canvas.bind("<Button-1>", self.on_map_click)

        # Tắt hoàn toàn sự kiện chuột mặc định của bản đồ
        for event in ("<B1-Motion>", "<ButtonRelease-1>", "<MouseWheel>"):
            self.map_widget.canvas.unbind(event)

    def on_map_click(self, event):
        lat, lon = self.map_widget.convert_canvas_coords_to_decimal_coords(event.x, event.y)
        node_id = f"node_{len(self.graph.nodes)}"
        self.graph.add_node(node_id, lat, lon)

        marker = self.map_widget.set_marker(lat, lon, text=node_id)
        self.markers.append(marker)

        if not self.start_node:
            self.start_node = node_id
            marker.set_text("Start")
        elif not self.goal_node:
            self.goal_node = node_id
            marker.set_text("Goal")
            # Auto add edges for demo (bidirectional)
            distance = self.distance(self.start_node, self.goal_node)
            self.graph.add_edge(self.start_node, self.goal_node, distance)
            self.graph.add_edge(self.goal_node, self.start_node, distance)

    def run_algorithm(self):
        if not self.start_node or not self.goal_node:
            return

        algo_name = self.alg_selector.get()
        algorithm_creator = self.ALGORITHMS.get(algo_name)
        if not algorithm_creator:
            return
            
        algo = algorithm_creator(self)
        path = algo.run(self.start_node, self.goal_node, self.graph)
        
        if path:
            coords = [self.graph.nodes[n] for n in path]
            self.map_widget.set_path(coords)

    def clear_selection(self):
        for marker in self.markers:
            marker.delete()
        self.markers.clear()
        self.start_node = None
        self.goal_node = None
        self.map_widget.delete_all_path()

    def distance(self, u, v):
        lat1, lon1 = self.graph.nodes[u]
        lat2, lon2 = self.graph.nodes[v]
        return math.sqrt((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2)

    def on_closing(self, event=None):
        self.destroy()

if __name__ == '__main__':
    app = App()
    app.mainloop()
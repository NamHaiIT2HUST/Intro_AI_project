import threading
from PIL import Image, ImageTk

class ObstacleManager:
    def __init__(self, app):
        self.app = app
        self.region_stacks = []

    def add_area_obstacles_async(self, lat1, lon1, lat2, lon2):
        threading.Thread(
            target=self._process_area, args=(lat1, lon1, lat2, lon2), daemon=True
        ).start()

    def _process_area(self, lat1, lon1, lat2, lon2):
        lat_min, lat_max = min(lat1, lat2), max(lat1, lat2)
        lon_min, lon_max = min(lon1, lon2), max(lon1, lon2)
        affected = [
            node for node, (lat, lon) in self.app.graph.node_coords.items()
            if lat_min <= lat <= lat_max and lon_min <= lon <= lon_max
            and not self.app.graph.is_obstacle(node)
        ]
        self.region_stacks.append(affected)
        self.app.graph.add_obstacles(affected)
        self.app.map_widget.after(0, lambda: self._add_area_marker(lat1, lon1, lat2, lon2))

    def _add_area_marker(self, lat1, lon1, lat2, lon2):
        lat_c = (lat1 + lat2) / 2
        lon_c = (lon1 + lon2) / 2
        img = Image.open("res\\house-flood-water-solid.png").resize((30, 30), Image.Resampling.LANCZOS)
        icon = ImageTk.PhotoImage(img)
        marker = self.app.map_widget.set_marker(lat_c, lon_c, text="Vùng cấm", icon=icon)
        self.app.set_marker_color(marker, "black")
        self.app.markers.append(marker)
        self.app.map_widget.delete_all_path()
        if self.app.start_node and self.app.goal_node:
            self.app.after(600, self.app.run_algorithm_thread)  # Delay chút để tránh lag

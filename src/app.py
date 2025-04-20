from tkintermapview import TkinterMapView
import customtkinter
from graph import Graph
from algorithm import *
import time
import osmnx as ox
from functools import lru_cache
import threading
from PIL import Image, ImageTk
from obstacle_manager import ObstacleManager


class App(customtkinter.CTk):
    APP_NAME = "Map View - Kim Mã, Ba Đình"
    CENTER_LAT, CENTER_LON =21.030085 ,105.824575
    source_path = "/Intro_AI/res/KimMa.osm"
    ALGORITHMS = {
        "A*": lambda self: AStar(self.distance),
        "Dijkstra": lambda _: Dijkstra(),
        "Greedy": lambda self: Greedy(self.distance),
        "BFS": lambda _: BFS(),
        "DFS": lambda _: DFS(),
        "Bellman-Ford": lambda _: BellmanFord(),
        "Bidirectional A*": lambda self: BidirectionalAStar(self.distance)
    }
    
    def __init__(self):
        super().__init__()
        self.title(self.APP_NAME)
        self.geometry("1000x700")
        self.resizable(True, True)  # Cho phép điều chỉnh kích thước cửa sổ

        # Khởi tạo biến instance
        self.graph = Graph()
        self.start_node = None
        self.goal_node = None
        self.markers = []
        self.path_line = None
        self.loading_indicator = None
        self.g = None  # Sẽ được tải trong phương thức load_graph
        self.obstacles=[]   #Danh sách các vật cản
        self.remove_obstacle_mode= False
        self.obstacle_stack=[] # Ngan xep luu vat can
        self.is_drawing_area = False  # Trạng thái vẽ vùng cấm
        self.region_start_canvas_coords = None
        self.region_rectangle = None
        self.obstacle_manager = ObstacleManager(self)

        # Thiết lập giao diện và bản đồ
        self._setup_ui()
        self._initialize_map()
        
        # Đăng ký sự kiện đóng cửa sổ
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Tải dữ liệu bản đồ trong một luồng riêng biệt
        self.load_map_thread = threading.Thread(target=self.load_graph)
        self.load_map_thread.daemon = True
        self.load_map_thread.start()

    def _setup_ui(self):
        # Tạo layout chính
        self.grid_columnconfigure(0, weight=5)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        # Bản đồ
        self.map_frame = customtkinter.CTkFrame(self, corner_radius=0)
        self.map_frame.grid(row=0, column=0, sticky="nsew")
        self.map_frame.grid_rowconfigure(0, weight=1)
        self.map_frame.grid_columnconfigure(0, weight=1)
        
        self.map_widget = TkinterMapView(self.map_frame, corner_radius=0)
        self.map_widget.grid(row=0, column=0, sticky="nsew")

        # Control Panel
        self.panel = customtkinter.CTkFrame(self)
        self.panel.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # Thiết lập các widget trong panel
        padding = 9
        
        self.title_label = customtkinter.CTkLabel(
            self.panel, 
            text="Tìm đường đi",
            font=customtkinter.CTkFont(size=18, weight="bold")
        )
        self.title_label.pack(pady=(15, padding))
        
        self.instruction_label = customtkinter.CTkLabel(
            self.panel,
            text="Chọn 2 điểm trên bản đồ\nđể thiết lập điểm đầu và điểm đích",
            font=customtkinter.CTkFont(size=12),
            justify="center"
        )
        self.instruction_label.pack(pady=padding)
        
        self.status_label = customtkinter.CTkLabel(
            self.panel,
            text="Đang tải bản đồ...",
            font=customtkinter.CTkFont(size=12),
            text_color="orange"
        )
        self.status_label.pack(pady=padding)

        self.status_label_obstacle = customtkinter.CTkLabel(
            self.panel,
            text="Chế độ vật cản: Tắt",
            font=customtkinter.CTkFont(size=12),
            text_color="red"
        )
        self.status_label_obstacle.pack(pady=padding)

        self.alg_label = customtkinter.CTkLabel(self.panel, text="Thuật toán:")
        self.alg_label.pack(pady=(5, 5))

        self.alg_selector = customtkinter.CTkComboBox(
            self.panel, values=list(self.ALGORITHMS.keys())
        )
        self.alg_selector.pack(pady=padding)

        self.run_button = customtkinter.CTkButton(
            self.panel, 
            text="Tìm đường", 
            command=self.run_algorithm_thread,
            state="disabled"
        )
        self.run_button.pack(pady=padding)

        # Thêm button để đặt vật cản
        self.obstacle_button = customtkinter.CTkButton(
             self.panel, 
             text="Đặt Vật Cản", 
             command=self.toggle_obstacle_mode,
             
         )
        self.obstacle_button.pack(pady=10)   

        self.remove_region_button = customtkinter.CTkButton(
            self.panel,
            text="Xóa vùng cấm",
            command=self.remove_last_region
        )
        self.remove_region_button.pack(pady=10)

        
        self.select_area_button = customtkinter.CTkButton(self.panel, text="Chọn vùng cấm", command=self.toggle_region_draw)
        self.select_area_button.pack(pady=10)
        
        # Them nut xoa vat can
        self.remove_obstacle_button=customtkinter.CTkButton(
            self.panel,
            text="Xóa vật cản",
            command=self.remove_obstacle,
            fg_color="#D35B58", 
            hover_color="#C77C78"
        )
        self.remove_obstacle_button.pack(pady=10)

        self.clear_button = customtkinter.CTkButton(
            self.panel, 
            text="Xoá chọn", 
            command=self.clear_selection,
            fg_color="#D35B58", 
            hover_color="#C77C78"
        )
        self.clear_button.pack(pady=padding)
        
        # Thêm widget hiển thị thông tin về đường đi
        self.info_frame = customtkinter.CTkFrame(self.panel)
        self.info_frame.pack(pady=padding, fill="x", padx=padding)
        
        self.distance_label = customtkinter.CTkLabel(
            self.info_frame,
            text="Khoảng cách: N/A",
            anchor="w"
        )
        self.distance_label.pack(pady=5, anchor="w")
        
        self.nodes_label = customtkinter.CTkLabel(
            self.info_frame,
            text="Số nút đã duyệt: N/A",
            anchor="w"
        )
        self.nodes_label.pack(pady=5, anchor="w")

        self.time_label = customtkinter.CTkLabel(
            self.info_frame,
            text="Thời gian tìm kiếm: N/A",
            anchor="w"
        )
        self.time_label.pack(pady=5, anchor="w")
        
    def toggle_obstacle_mode(self):
        self.obstacle_mode=not getattr(self,'obstacle_mode',False)
        if self.obstacle_mode:
            self.status_label_obstacle.configure(text="Chế độ vật cản: Bật",text_color="green")
        else:
            self.status_label_obstacle.configure(text="Chế độ vật cản: Tắt",text_color="red")

    def toggle_region_draw(self):
        """Bật/tắt chế độ vẽ vùng cấm"""
        self.is_drawing_area = not self.is_drawing_area  # Chuyển đổi trạng thái
        if self.is_drawing_area:
            self.select_area_button.configure(text="Hủy chọn vùng cấm")  # Thay đổi tên nút
            # Cập nhật sự kiện để vẽ vùng cấm
            self.map_widget.canvas.bind("<ButtonPress-3>", self.on_region_draw_start)
            self.map_widget.canvas.bind("<B3-Motion>", self.on_region_draw_motion)
            self.map_widget.canvas.bind("<ButtonRelease-3>", self.on_region_draw_end)
        else:
            self.select_area_button.configure(text="Chọn vùng cấm")  # Đặt lại tên nút
            # Hủy bỏ các sự kiện vẽ vùng cấm
            self.map_widget.canvas.unbind("<ButtonPress-3>")
            self.map_widget.canvas.unbind("<B3-Motion>")
            self.map_widget.canvas.unbind("<ButtonRelease-3>")

    def _initialize_map(self):
        self.map_widget.set_position(self.CENTER_LAT, self.CENTER_LON)
        self.map_widget.set_zoom(17)
        self.map_widget.add_right_click_menu_command("Đặt điểm đầu", self.set_start_marker)
        self.map_widget.add_right_click_menu_command("Đặt điểm đích", self.set_goal_marker)
        
        # Giữ nguyên sự kiện chuột để điều hướng bản đồ
        self.map_widget.canvas.bind("<Button-1>", self.on_map_click)

        # Tắt hoàn toàn sự kiện chuột mặc định của bản đồ
        for event in ("<B1-Motion>", "<ButtonRelease-1>"):
            self.map_widget.canvas.unbind(event)

    def on_map_click(self, event):
        
        if getattr(self, 'obstacle_mode', False):
            self.add_obstacle((event.x, event.y))
            return
        # Nếu đã có cả điểm đầu và điểm đích, không cho phép thêm điểm mới
        if self.start_node and self.goal_node:
            return None 
        lat, lon = self.map_widget.convert_canvas_coords_to_decimal_coords(event.x, event.y)
        
        if not self.g:
            return  # Bản đồ chưa được tải xong
            
        print(f"Clicked at: {lat}, {lon}")
        node = self.find_nearest_node(lat, lon)
        print(f"Nearest node: {node}") 

        marker = self.map_widget.set_marker(lat, lon)
        self.markers.append(marker)
        if not self.start_node:
            self.start_node = node
            marker.set_text("Điểm đầu")
            # Đặt màu cho marker thay vì sử dụng icon
            if hasattr(marker, "canvas_id"):
                self.map_widget.canvas.itemconfig(marker.canvas_id, fill="green")
        elif not self.goal_node:
            self.goal_node = node
            marker.set_text("Điểm đích")
            # Đặt màu cho marker thay vì sử dụng icon
            if hasattr(marker, "canvas_id"):
                self.map_widget.canvas.itemconfig(marker.canvas_id, fill="red")
            
        self.update_run_button()
    
    def set_start_marker(self, coords):
        if self.start_node:
            # Xóa marker cũ nếu đã có
            for marker in self.markers:
                if hasattr(marker, "text") and marker.text == "Điểm đầu":
                    self.markers.remove(marker)
                    marker.delete()
        
        lat, lon = coords
        if not self.g:
            return  # Bản đồ chưa được tải xong
            
        node = self.find_nearest_node(lat, lon)
        
        marker = self.map_widget.set_marker(lat, lon, text="Điểm đầu")
        # Đặt màu cho marker thay vì sử dụng icon
        if hasattr(marker, "canvas_id"):
            self.map_widget.canvas.itemconfig(marker.canvas_id, fill="green")
        self.markers.append(marker)
        self.start_node = node
        
        self.update_run_button()
    
    def set_goal_marker(self, coords):
        if self.goal_node:
            # Xóa marker cũ nếu đã có
            for marker in self.markers:
                if hasattr(marker, "text") and marker.text == "Điểm đích":
                    self.markers.remove(marker)
                    marker.delete()
        
        lat, lon = coords
        if not self.g:
            return  # Bản đồ chưa được tải xong
            
        node = self.find_nearest_node(lat, lon)
        
        marker = self.map_widget.set_marker(lat, lon, text="Điểm đích")
        # Đặt màu cho marker thay vì sử dụng icon
        if hasattr(marker, "canvas_id"):
            self.map_widget.canvas.itemconfig(marker.canvas_id, fill="red")
        self.markers.append(marker)
        self.goal_node = node
        
        self.update_run_button()
    
    def set_marker_color(self, marker, color):
        """Thiết lập màu cho marker nếu có thể"""
        if hasattr(marker, "canvas_id"):
            self.map_widget.canvas.itemconfig(marker.canvas_id, fill=color)
    
    def update_run_button(self):
        if self.start_node and self.goal_node and self.g:
            self.run_button.configure(state="normal")
        else:
            self.run_button.configure(state="disabled")

    def load_graph(self):
        try:
            # Tải dữ liệu đồ thị từ file OSM
            self.g = ox.graph_from_xml(self.source_path, retain_all=True, simplify=False)
            
            # Chuyển đổi sang đồ thị nội bộ
            for node_id, data in self.g.nodes(data=True):
                self.graph.add_node(node_id, data['y'], data['x'])
            
            for u, v, data in self.g.edges(data=True):
                self.graph.add_edge(u, v, data['length'])
            
            # Cập nhật giao diện sau khi tải xong
            self.after(0, self.on_graph_loaded)
        except Exception as e:
            # Xử lý lỗi và thông báo cho người dùng
            print(f"Lỗi khi tải bản đồ: {e}")
            self.after(0, lambda: self.status_label.configure(
                text=f"Lỗi tải bản đồ: {str(e)[:50]}...", 
                text_color="red"
            ))
    
    def on_graph_loaded(self):
        self.status_label.configure(text="Bản đồ đã sẵn sàng", text_color="green")
        self.update_run_button()
    
    @lru_cache(maxsize=128)
    def find_nearest_node(self, lat, lon):
        """Tìm nút gần nhất với tọa độ đã cho, với cache để tăng hiệu suất"""
        return self.graph.find_nearest_node_within_radius(lat, lon, initial_radius=10, step=10, max_radius=1000)

    def run_algorithm_thread(self):
        """Khởi chạy thuật toán trong một luồng riêng biệt"""
        if not self.start_node or not self.goal_node:
            return
            
        # Hiển thị trạng thái đang tìm đường
        self.status_label.configure(text="Đang tìm đường...", text_color="orange")
        self.run_button.configure(state="disabled")
        
        # Khởi chạy thuật toán trong một luồng riêng
        thread = threading.Thread(target=self.run_algorithm)
        thread.daemon = True
        thread.start()

    def run_algorithm(self):
        try:
            algo_name = self.alg_selector.get()
            algorithm_creator = self.ALGORITHMS.get(algo_name)
            if not algorithm_creator:
                self.after(0, lambda: self.status_label.configure(
                    text=f"Không tìm thấy thuật toán: {algo_name}", 
                    text_color="red"
                ))
                return
                
            algo = algorithm_creator(self)
            # Cần điều chỉnh hàm run() trong các lớp thuật toán để trả về cả path và stats
            # Nhưng hiện tại lớp thuật toán có thể chỉ trả về path, nên chúng ta cần xử lý cả hai trường hợp
            time_start = time.time()
            result = algo.run(self.start_node, self.goal_node, self.graph)
            time_total = (time.time() - time_start) * 1000
            
            # Kiểm tra kết quả trả về
            if isinstance(result, tuple) and len(result) == 2:
                count_nodes, path = result
                stats = {"distance": self.calculate_path_distance(path) if path else 0, "expanded_nodes": count_nodes, "time": time_total}
            else:
                path = result
                stats = {"distance": self.calculate_path_distance(path) if path else 0, "expanded_nodes": "N/A", "time": time_total}
            
            if path:
                # Cập nhật UI trên luồng chính
                self.after(0, lambda: self.draw_path(path, stats))
                self.after(0, lambda: self.status_label.configure(
                    text=f"Đã tìm thấy đường đi ({algo_name})", 
                    text_color="green"
                ))
            else:
                self.after(0, lambda: self.status_label.configure(
                    text="Không tìm thấy đường đi", 
                    text_color="red"
                ))
        except Exception as e:
            
            error_message = f"Lỗi: {str(e)[:50]}..."
            print(error_message)
            self.after(2000, lambda lbl=self.status_label, msg=error_message: lbl.configure(text=msg))
        
        # Kích hoạt lại nút tìm đường
        self.after(0, lambda: self.run_button.configure(state="normal"))

    def clear_selection(self):
        """Xóa tất cả các điểm đã chọn và đường đi"""
        for marker in self.markers:
            marker.delete()
        self.markers.clear()
        self.start_node = None
        self.goal_node = None
        self.map_widget.canvas.delete(self.region_rectangle)
        self.region_rectangle = None
        self.map_widget.delete_all_path()
        # Đặt lại nhãn thông tin
        self.distance_label.configure(text="Khoảng cách: N/A")
        self.nodes_label.configure(text="Số nút đã duyệt: N/A")
        self.time_label.configure(text="Thời gian tìm kiếm: N/A")
        self.status_label.configure(text="Bản đồ đã sẵn sàng", text_color="green")
        
        # Cập nhật trạng thái nút tìm đường
        self.update_run_button()

    @lru_cache(maxsize=1024)
    def distance(self, u, v):
        """Tính khoảng cách giữa hai nút, với cache để tăng hiệu suất"""
        return self.graph.heuristic(u, v)  # Sử dụng hàm heuristic của đồ thị
        
    def calculate_path_distance(self, path):
        """Tính tổng khoảng cách của đường đi"""
        if not path or len(path) < 2:
            return 0
            
        total_distance = 0
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            total_distance += self.distance(u, v)

        return total_distance

    def on_closing(self, event=None):
        # Đảm bảo các luồng con dừng lại khi đóng ứng dụng
        self.quit()
        self.destroy()

    def draw_path(self, path, stats=None):
        if not path:
            return
            
        # Vẽ đường đi trên bản đồ
        coords = [self.graph.nodes[n] for n in path]
        self.map_widget.delete_all_path()
        self.path_line = self.map_widget.set_path(coords, color="blue", width=5)
        
        # Cập nhật thông tin về đường đi
        if stats:
            self.distance_label.configure(text=f"Khoảng cách: {stats.get('distance', 'N/A'):.2f} m")
            self.nodes_label.configure(text=f"Số nút đã duyệt: {stats.get('expanded_nodes', 'N/A')}")
            self.time_label.configure(text=f"Thời gian tìm kiếm: {stats.get('time', 'N/A'):.3f} ms")
            
    #Them vat can
    def add_obstacle(self,coords):
        lat,lon= self.map_widget.convert_canvas_coords_to_decimal_coords(*coords) if coords else (None,None)
        if lat is None or lon is None:
            return 
        
        node =self.find_nearest_node(lat,lon)
        if node not in self.obstacles:
            self.obstacles.append(node)
            self.graph.add_obstacle(node)
            self.obstacle_stack.append(node)
            obstacle_icon = Image.open("res\\obstacle.png")
            obstacle_icon = obstacle_icon.resize((30, 30),Image.Resampling.LANCZOS)
            obstacle_icon = ImageTk.PhotoImage(obstacle_icon)
            marker=self.map_widget.set_marker(lat,lon,text="Vật cản",icon=obstacle_icon)
            self.set_marker_color(marker,"black")
            self.markers.append(marker)
            self.map_widget.delete_all_path()
            if self.start_node and self.goal_node:
                self.run_algorithm_thread()

    #Xoa vat can
    def remove_obstacle(self):
        if not self.obstacle_stack:
            return
        last_obstacle_node=self.obstacle_stack.pop()
        self.obstacles.remove(last_obstacle_node)
        self.graph.remove_obstacle(last_obstacle_node)

        #Xoa marker cua vat can tren ban do
        for i in range(len(self.markers)-1, -1, -1):
            marker = self.markers[i]
            if hasattr(marker,"text") and marker.text =="Vật cản":
                self.markers.remove(marker)
                marker.delete()
                break; # Xoa marker cua vat can gan nhat
        self.map_widget.delete_all_path()
        if self.start_node and self.goal_node:
            self.run_algorithm_thread()

    #Vung cam
    def on_region_draw_start(self,event):
        self.region_start_canvas_coords=(event.x,event.y)
        self.region_rectangle=self.map_widget.canvas.create_rectangle(
            event.x,event.y,event.x,event.y,
            outline="red",width=2,dash=(4,2)
        )
        
    def on_region_draw_motion(self,event):
        if(self.region_rectangle):
            self.map_widget.canvas.coords(self.region_rectangle,
                                         self.region_start_canvas_coords[0],
                                         self.region_start_canvas_coords[1],
                                         event.x,event.y)
    
    def on_region_draw_end(self, event):
        if not self.region_rectangle:
            return

        x1, y1 = self.region_start_canvas_coords
        x2, y2 = event.x, event.y
        lat1, lon1 = self.map_widget.convert_canvas_coords_to_decimal_coords(x1, y1)
        lat2, lon2 = self.map_widget.convert_canvas_coords_to_decimal_coords(x2, y2)

        self.obstacle_manager.add_area_obstacles_async(lat1, lon1, lat2, lon2)
        self.map_widget.canvas.delete(self.region_rectangle)
        self.region_rectangle = None

    #Xoa vung cam
    def remove_last_region(self):
        if not self.obstacle_manager.region_stacks:
            return
        last_region = self.obstacle_manager.region_stacks.pop()  # danh sách node
        for node in last_region:
            self.graph.remove_obstacle(node)
            # nếu bạn lưu marker vùng cấm, cũng cần xóa marker tương ứng ở đây
        # Xoá đường và chạy lại thuật toán nếu cần
        self.map_widget.delete_all_path()
        #xóa icon vùng cấm trên bản đồ
        for marker in self.markers:
            if hasattr(marker, "text") and marker.text == "Vùng cấm":
                self.markers.remove(marker)
                marker.delete()
                break
        if self.start_node and self.goal_node:
            self.run_algorithm_thread()

if __name__ == '__main__':
    customtkinter.set_appearance_mode("System")  # Hỗ trợ chế độ giao diện hệ thống
    customtkinter.set_default_color_theme("blue")  # Đặt chủ đề màu sắc
    
    app = App()
    app.mainloop()
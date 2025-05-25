import customtkinter
from app import App

if __name__ == '__main__':
    customtkinter.set_appearance_mode("System")  # Hỗ trợ chế độ giao diện hệ thống
    customtkinter.set_default_color_theme("blue")  # Đặt chủ đề màu sắc
    app = App()
    app.mainloop()
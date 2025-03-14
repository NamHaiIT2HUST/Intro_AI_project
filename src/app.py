from tkinter import messagebox
from tkintermapview import TkinterMapView
from tkinter import Tk
import customtkinter
import networkx as nx

class App(customtkinter.CTk):
    App_Name = "Map View"
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.title(self.App_Name)
        self.geometry("800x600")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.createcommand("exit", self.on_closing)

        self.market = []
        self.graph = nx.Graph()

    def on_closing(self, event=None):
        self.destroy()

        


if __name__ == '__main__':
    app = App()
    app.mainloop()

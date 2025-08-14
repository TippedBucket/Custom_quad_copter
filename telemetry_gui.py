import tkinter as tk
from tkinter import font as tkFont
from tkintermapview import TkinterMapView
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque
import threading
import time
import random

class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Telemetry GUI")
        self.root.geometry("1200x800")

        # Base font
        self.base_font_size = 12
        self.font = tkFont.Font(family="Arial", size=self.base_font_size, weight="bold")

        self.follow_drone = tk.BooleanVar(value=True)

        # Map
        self.map_widget = TkinterMapView(self.root, width=600, height=400, corner_radius=0)
        self.map_widget.grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")
        self.map_widget.set_zoom(16)
        self.drone_marker = self.map_widget.set_marker(0, 0, text="Drone")

        # Follow toggle
        self.follow_check = tk.Checkbutton(self.root, text="Follow Drone", variable=self.follow_drone, font=self.font)
        self.follow_check.grid(row=1, column=0, sticky="w", padx=10)

        # Telemetry labels
        self.telemetry_labels = {}
        fields = ["Latitude", "Longitude", "Battery (%)", "Voltage (V)", "Current (A)",
                  "Pitch (°)", "Roll (°)", "Yaw (°)", "Throttle", "Flight Time (s)"]
        for i, field in enumerate(fields):
            label = tk.Label(self.root, text=f"{field}: ---", font=self.font)
            label.grid(row=2 + i, column=0, sticky="w", padx=10, pady=4)
            self.telemetry_labels[field] = label

        # Plots
        self.fig, self.axs = plt.subplots(3, 1, figsize=(5, 4))
        self.fig.tight_layout()
        self.pitch_data = deque([0]*50, maxlen=50)
        self.roll_data = deque([0]*50, maxlen=50)
        self.yaw_data = deque([0]*50, maxlen=50)
        self.time_data = deque(range(-49, 1), maxlen=50)

        self.lines = [
            self.axs[0].plot(self.time_data, self.pitch_data)[0],
            self.axs[1].plot(self.time_data, self.roll_data)[0],
            self.axs[2].plot(self.time_data, self.yaw_data)[0]
        ]
        for ax, title in zip(self.axs, ["Pitch", "Roll", "Yaw"]):
            ax.set_ylim(-180, 180)
            ax.set_title(title)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=2, column=1, rowspan=10, padx=10, pady=10, sticky="nsew")

        # Allow resizing
        for i in range(12):
            self.root.grid_rowconfigure(i, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=2)

        # Handle dynamic resizing
        self.root.bind("<Configure>", self.on_resize)

        # Start telemetry thread
        self.running = True
        self.thread = threading.Thread(target=self.read_telemetry)
        self.thread.daemon = True
        self.thread.start()

    def on_resize(self, event):
        # Dynamically scale font size based on window height
        new_size = max(10, int(self.root.winfo_height() / 50))
        if new_size != self.font.cget("size"):
            self.font.configure(size=new_size)

    def update_gui(self, data):
        try:
            lat, lon = float(data["lat"]), float(data["lon"])
            self.drone_marker.set_position(lat, lon)
            if self.follow_drone.get():
                self.map_widget.set_position(lat, lon)

            self.telemetry_labels["Latitude"].config(text=f"Latitude: {lat:.6f}")
            self.telemetry_labels["Longitude"].config(text=f"Longitude: {lon:.6f}")
            self.telemetry_labels["Battery (%)"].config(text=f"Battery (%): {data['battery']}")
            self.telemetry_labels["Voltage (V)"].config(text=f"Voltage (V): {data['voltage']}")
            self.telemetry_labels["Current (A)"].config(text=f"Current (A): {data['current']}")
            self.telemetry_labels["Pitch (°)"].config(text=f"Pitch (°): {data['pitch']}")
            self.telemetry_labels["Roll (°)"].config(text=f"Roll (°): {data['roll']}")
            self.telemetry_labels["Yaw (°)"].config(text=f"Yaw (°): {data['yaw']}")
            self.telemetry_labels["Throttle"].config(text=f"Throttle: {data['throttle']}")
            self.telemetry_labels["Flight Time (s)"].config(text=f"Flight Time (s): {data['time']}")

            self.pitch_data.append(float(data["pitch"]))
            self.roll_data.append(float(data["roll"]))
            self.yaw_data.append(float(data["yaw"]))
            self.time_data.append(self.time_data[-1] + 1)

            self.lines[0].set_ydata(self.pitch_data)
            self.lines[1].set_ydata(self.roll_data)
            self.lines[2].set_ydata(self.yaw_data)
            for line, ax in zip(self.lines, self.axs):
                line.set_xdata(self.time_data)
                ax.relim()
                ax.autoscale_view()

            self.canvas.draw()
        except Exception as e:
            print("Update error:", e)

    def read_telemetry(self):
        t = 0
        while self.running:
            simulated_data = {
                "lat": 49.2606 + 0.0001 * random.uniform(-1, 1),
                "lon": -123.2460 + 0.0001 * random.uniform(-1, 1),
                "battery": round(70 + 5 * random.uniform(-1, 1), 2),
                "voltage": round(11.1 + random.uniform(-0.1, 0.1), 2),
                "current": round(2 + random.uniform(-0.5, 0.5), 2),
                "pitch": round(random.uniform(-20, 20), 2),
                "roll": round(random.uniform(-20, 20), 2),
                "yaw": round(random.uniform(-180, 180), 2),
                "throttle": round(1500 + random.uniform(-300, 300), 2),
                "time": t
            }
            self.root.after(0, self.update_gui, simulated_data)
            t += 1
            time.sleep(1)

    def close(self):
        self.running = False
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()

"""BronzeLink ground station with a live GPS map and satellite toggle.

Install: python -m pip install tkintermapview
Connect to BronzeLink Wi-Fi, then: python bronzelink_groundstation_map.py

Map tiles require internet access initially; cached tiles may work offline.
"""
import json
import math
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import tkintermapview
except ImportError:
    raise SystemExit("Missing dependency. Run: python -m pip install tkintermapview")

PORT = 14560
PICO_IP = "192.168.4.1"
DISCOVERY_PORT = 14561
HELLO_INTERVAL = 3.0
LINK_TIMEOUT = 5.0
MAX_TRACK_POINTS = 1000

MAP_TILES = "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png"
SATELLITE_TILES = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/"
    "World_Imagery/MapServer/tile/{z}/{y}/{x}"
)


def fmt(value, digits=1, suffix=""):
    if value is None:
        return "---"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}{suffix}"
    return str(value)


def valid_coordinate(lat, lon):
    return (isinstance(lat, (int, float)) and not isinstance(lat, bool)
            and isinstance(lon, (int, float)) and not isinstance(lon, bool)
            and math.isfinite(lat) and math.isfinite(lon)
            and -90 <= lat <= 90 and -180 <= lon <= 180
            and not (lat == 0 and lon == 0))


class BronzeLinkApp:
    def __init__(self, root):
        self.root = root
        root.title("BronzeLink Ground Station")
        root.geometry("1100x720")
        self.events = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()
        self.track = []
        self.marker = None
        self.path = None
        self.follow = tk.BooleanVar(value=True)
        self.satellite = tk.BooleanVar(value=False)
        self.last_packet = None
        self.count = 0
        self.values = {}

        panel = ttk.Frame(root, padding=12)
        panel.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(panel, text="BronzeLink", font=("Segoe UI", 19, "bold")).pack(anchor="w", pady=(0, 12))
        fields = [
            ("status", "Link"), ("flight_mode", "Flight mode"),
            ("latitude", "Latitude"), ("longitude", "Longitude"),
            ("heading_deg", "Heading"), ("home_distance_m", "Home distance"),
            ("altitude_m", "Altitude"), ("battery_voltage_v", "Battery voltage"),
            ("battery_current_a", "Battery current"),
            ("battery_remaining_pct", "Battery remaining"),
            ("packets", "Packets received"),
        ]
        for key, label in fields:
            ttk.Label(panel, text=label + ":").pack(anchor="w", pady=(5, 0))
            var = tk.StringVar(value="---")
            ttk.Label(panel, textvariable=var, font=("Segoe UI", 11, "bold")).pack(anchor="w")
            self.values[key] = var
        ttk.Checkbutton(panel, text="Follow aircraft", variable=self.follow).pack(anchor="w", pady=(20, 8))
        ttk.Checkbutton(panel, text="Satellite view", variable=self.satellite,
                        command=self.toggle_satellite).pack(anchor="w", pady=(0, 8))
        ttk.Button(panel, text="Clear flight track", command=self.clear_track).pack(anchor="w")
        ttk.Label(panel, text="Map tiles need internet initially.\nGPS tracking still works offline.",
                  justify="left", wraplength=220).pack(anchor="w", pady=(25, 0))

        map_frame = ttk.Frame(root)
        map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.map = tkintermapview.TkinterMapView(map_frame, corner_radius=0)
        self.map.pack(fill=tk.BOTH, expand=True)
        self.map.set_tile_server(MAP_TILES, max_zoom=19)
        self.map.set_position(51.0, -2.5)  # Initial view only; not aircraft position
        self.map.set_zoom(9)

        self.worker = threading.Thread(target=self.receive_loop, daemon=True)
        self.worker.start()
        self.root.after(100, self.process_events)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def receive_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", PORT))
            sock.settimeout(0.5)
            last_hello = -1e9
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now - last_hello >= HELLO_INTERVAL:
                    try:
                        sock.sendto(b"BRONZELINK_HELLO", (PICO_IP, DISCOVERY_PORT))
                    except OSError as exc:
                        self.post(("error", str(exc)))
                    last_hello = now
                try:
                    payload, sender = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self.stop_event.is_set():
                        self.post(("error", str(exc)))
                    continue
                try:
                    data = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if isinstance(data, dict):
                    self.post(("telemetry", data))
        except OSError as exc:
            self.post(("error", str(exc)))
        finally:
            sock.close()

    def post(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            pass

    def process_events(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "telemetry":
                    self.update_telemetry(data)
                else:
                    self.values["status"].set("Network error: " + data)
        except queue.Empty:
            pass
        if self.last_packet is None or time.monotonic() - self.last_packet > LINK_TIMEOUT:
            self.values["status"].set("NO TELEMETRY")
        if not self.stop_event.is_set():
            self.root.after(100, self.process_events)

    def update_telemetry(self, data):
        self.last_packet = time.monotonic()
        self.count += 1
        link_ok = bool(data.get("link_ok"))
        self.values["status"].set("OK" if link_ok else "LOST")
        self.values["flight_mode"].set(str(data.get("flight_mode") or "---"))
        self.values["latitude"].set(fmt(data.get("latitude"), 7))
        self.values["longitude"].set(fmt(data.get("longitude"), 7))
        self.values["heading_deg"].set(fmt(data.get("heading_deg"), 1, "°"))
        self.values["home_distance_m"].set(fmt(data.get("home_distance_m"), 0, " m"))
        self.values["altitude_m"].set(fmt(data.get("altitude_m"), 1, " m"))
        self.values["battery_voltage_v"].set(fmt(data.get("battery_voltage_v"), 1, " V"))
        self.values["battery_current_a"].set(fmt(data.get("battery_current_a"), 1, " A"))
        self.values["battery_remaining_pct"].set(fmt(data.get("battery_remaining_pct"), 0, "%"))
        self.values["packets"].set(str(self.count))

        lat, lon = data.get("latitude"), data.get("longitude")
        if not link_ok or not valid_coordinate(lat, lon):
            return
        point = (float(lat), float(lon))
        if self.marker is None:
            self.marker = self.map.set_marker(*point, text="Aircraft")
            self.map.set_position(*point)
            self.map.set_zoom(16)
        else:
            self.marker.set_position(*point)
            if self.follow.get():
                self.map.set_position(*point)
        if not self.track or point != self.track[-1]:
            self.track.append(point)
            if len(self.track) > MAX_TRACK_POINTS:
                self.track.pop(0)
            if self.path is not None:
                self.path.delete()
                self.path = None
            if len(self.track) >= 2:
                self.path = self.map.set_path(self.track)

    def toggle_satellite(self):
        """Switch background imagery without affecting marker or flight track."""
        tiles = SATELLITE_TILES if self.satellite.get() else MAP_TILES
        self.map.set_tile_server(tiles, max_zoom=19)

    def clear_track(self):
        self.track.clear()
        if self.path is not None:
            self.path.delete()
            self.path = None

    def close(self):
        self.stop_event.set()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    BronzeLinkApp(root)
    root.mainloop()

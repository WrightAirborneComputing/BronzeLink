"""BronzeLink UDP JSON telemetry receiver (Windows/macOS/Linux, Python 3).

Connect the PC's Wi-Fi to BronzeLink first (password: BronzeLink123),
then run: python bronzelink_receiver.py
"""
import json
import socket
from datetime import datetime
import time

PORT = 14560
PICO_IP = "192.168.4.1"
DISCOVERY_PORT = 14561
HELLO_INTERVAL = 3.0


def fmt(value, digits=1, suffix=""):
    if value is None:
        return "---"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}{suffix}"
    return str(value)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(1.0)

    print(f"Listening for BronzeLink JSON telemetry on UDP port {PORT}...")
    print("Connect this PC to Wi-Fi SSID BronzeLink (password BronzeLink123).")
    print("Automatic receiver registration enabled (UDP 14561).")
    print("Press Ctrl+C to stop.\n")

    count = 0
    last_hello = -1e9
    last_packet = time.monotonic()
    last_wait_notice = -1e9
    try:
        while True:
            now = time.monotonic()
            if now - last_hello >= HELLO_INTERVAL:
                try:
                    sock.sendto(b"BRONZELINK_HELLO", (PICO_IP, DISCOVERY_PORT))
                except OSError as exc:
                    print("Receiver registration failed:", exc)
                last_hello = now
            try:
                payload, sender = sock.recvfrom(4096)
            except socket.timeout:
                now = time.monotonic()
                if now - last_packet >= 5 and now - last_wait_notice >= 5:
                    print("Waiting for telemetry... (check Wi-Fi connection and firewall)")
                    last_wait_notice = now
                continue
            try:
                data = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                print(f"Ignoring invalid JSON from {sender[0]}: {exc}")
                continue
            if not isinstance(data, dict):
                continue
            last_packet = time.monotonic()
            count += 1
            stamp = datetime.now().strftime("%H:%M:%S")
            link = "OK" if data.get("link_ok") else "LOST"
            print(f"[{stamp}] #{count} from {sender[0]} | LINK {link}")
            print("  GPS:       ", fmt(data.get("latitude"), 7), ",", fmt(data.get("longitude"), 7))
            print("  Heading:   ", fmt(data.get("heading_deg"), 1, " deg"))
            print("  Home dist: ", fmt(data.get("home_distance_m"), 0, " m"))
            print("  Altitude:  ", fmt(data.get("altitude_m"), 1, " m"))
            print("  Battery:   ", fmt(data.get("battery_voltage_v"), 1, " V"),
                  " | ", fmt(data.get("battery_current_a"), 1, " A"),
                  " | ", fmt(data.get("battery_remaining_pct"), 0, "%"))
            print()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()

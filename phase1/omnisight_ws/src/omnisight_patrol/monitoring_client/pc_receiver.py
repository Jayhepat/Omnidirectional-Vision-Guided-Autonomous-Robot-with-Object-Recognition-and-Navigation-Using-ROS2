#!/usr/bin/env python3
"""
pc_receiver.py — Omnisight Monitoring Client (runs on YOUR PC/Laptop)
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Run this on your PC BEFORE starting the robot:
  python3 pc_receiver.py

It will:
  - Listen for TCP connections from Omnisight robot
  - Receive alert JSON payloads (stranger / scene_change)
  - Display alert popup with captured image
  - Save all alerts to ./omnisight_alerts/ folder
  - Print alert log to terminal

Setup:
  1. Note your PC IP address (e.g. 192.168.0.100)
  2. Edit config/patrol_waypoints.yaml on the robot:
       monitoring_device_ip: "192.168.0.100"
  3. Run this script FIRST on PC, then launch robot

Protocol: 4-byte big-endian length header + JSON payload
"""
import socket, struct, json, base64, os, time, threading
from datetime import datetime

# ── CONFIG ────────────────────────────────────────────────────────────────
LISTEN_PORT  = 9999
SAVE_DIR     = "./omnisight_alerts"
SHOW_IMAGES  = True   # Set False if no display (headless server)

# Try to import display libs
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[WARNING] opencv-python not installed — images will be saved but not displayed")

os.makedirs(SAVE_DIR, exist_ok=True)

# ── COLOUR CODES ─────────────────────────────────────────────────────────
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def print_banner():
    print(f"""
{CYAN}{BOLD}
╔══════════════════════════════════════════════════════╗
║   OMNISIGHT Monitoring Client                        ║
║   VIT Bhopal University | SEEE Capstone 2022-2026    ║
╚══════════════════════════════════════════════════════╝
{RESET}""")

def recv_exact(sock, n):
    """Receive exactly n bytes from socket."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf

def handle_alert(payload: dict):
    """Process a received alert."""
    alert_type = payload.get("type", "unknown")
    ts         = payload.get("timestamp", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    img_b64    = payload.get("image_b64", "")

    if alert_type == "stranger_detected":
        dist = payload.get("distance", "?")
        print(f"\n{RED}{BOLD}⚠  STRANGER DETECTED{RESET}")
        print(f"   Time      : {ts}")
        print(f"   Distance  : {dist} (threshold: {payload.get('threshold','?')})")
        prefix = f"stranger_{ts.replace(':','').replace(' ','_')}"

    elif alert_type == "scene_change":
        zone   = payload.get("zone_label", "?")
        ztype  = payload.get("zone_type",  "?")
        ssim   = payload.get("ssim",       "?")
        diff   = payload.get("diff_pct",   "?")
        print(f"\n{YELLOW}{BOLD}⚠  SCENE CHANGE DETECTED{RESET}")
        print(f"   Time      : {ts}")
        print(f"   Zone      : {zone} (type: {ztype})")
        print(f"   SSIM      : {ssim}  |  Pixel diff: {diff}%")
        prefix = f"scene_{zone.replace(' ','_')}_{ts.replace(':','').replace(' ','_')}"

    else:
        print(f"\n{CYAN}[INFO] Alert received: {alert_type}{RESET}")
        prefix = f"alert_{ts.replace(':','').replace(' ','_')}"

    # Save image
    if img_b64:
        try:
            img_bytes = base64.b64decode(img_b64)
            img_path  = os.path.join(SAVE_DIR, f"{prefix}.jpg")
            with open(img_path, "wb") as f: f.write(img_bytes)
            print(f"   Image saved: {img_path}")

            # Display image
            if SHOW_IMAGES and CV2_AVAILABLE:
                nparr = np.frombuffer(img_bytes, dtype=np.uint8)
                img   = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is not None:
                    title = f"OMNISIGHT ALERT: {alert_type.upper()}"
                    cv2.putText(img, title, (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                    cv2.putText(img, ts, (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
                    cv2.imshow(title, img)
                    cv2.waitKey(3000)  # show for 3 seconds
                    cv2.destroyAllWindows()
        except Exception as e:
            print(f"   [Image error: {e}]")

    # Save full JSON log
    log_path = os.path.join(SAVE_DIR, f"{prefix}.json")
    with open(log_path, "w") as f:
        json.dump({k:v for k,v in payload.items() if k!="image_b64"}, f, indent=2)

def handle_client(conn, addr):
    """Handle one connection from Omnisight robot."""
    print(f"{GREEN}[+] Omnisight robot connected from {addr}{RESET}")
    try:
        while True:
            # Read 4-byte length header
            header = recv_exact(conn, 4)
            length = struct.unpack(">I", header)[0]
            if length > 10 * 1024 * 1024:  # sanity: max 10MB
                print(f"[WARN] Payload too large ({length} bytes), skipping")
                continue
            # Read payload
            data    = recv_exact(conn, length)
            payload = json.loads(data.decode("utf-8"))
            handle_alert(payload)
    except (ConnectionError, ConnectionResetError):
        print(f"[INFO] Robot disconnected from {addr}")
    except Exception as e:
        print(f"[ERROR] Client handler: {e}")
    finally:
        conn.close()

def main():
    print_banner()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", LISTEN_PORT))
    server.listen(5)

    # Get local IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except:
        local_ip = "your-pc-ip"

    print(f"{GREEN}Listening on port {LISTEN_PORT}{RESET}")
    print(f"{BOLD}Your PC IP: {local_ip}{RESET}")
    print(f"Set this in robot config: monitoring_device_ip: \"{local_ip}\"")
    print(f"Alerts saved to: {os.path.abspath(SAVE_DIR)}/")
    print(f"Waiting for Omnisight robot...\n")

    try:
        while True:
            conn, addr = server.accept()
            t = threading.Thread(target=handle_client, args=(conn,addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print(f"\n{CYAN}Omnisight monitoring client stopped.{RESET}")
    finally:
        server.close()

if __name__ == "__main__":
    main()

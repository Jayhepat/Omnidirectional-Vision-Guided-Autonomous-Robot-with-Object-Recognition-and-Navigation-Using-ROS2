#!/usr/bin/env python3
"""
alert_manager.py — Omnisight Alert Manager
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Receives alerts from:
  - face_recognition_node  → /omnisight/alert/stranger
  - scene_monitor          → /omnisight/alert/scene_change
  - buzzer trigger         → /omnisight/trigger_buzzer

Actions:
  1. Send alert image + metadata to monitoring PC via WiFi TCP socket
  2. Activate GPIO buzzer for stranger detections
  3. Save alert image locally to log directory
  4. Log all events with timestamp

TCP protocol (Module 8 — WiFi Socket):
  4-byte length (big-endian) + JSON payload bytes
  Payload includes image as base64 JPEG
"""
import rclpy, os, yaml, json, time, socket, struct, threading, base64
from rclpy.node import Node
from std_msgs.msg import String, Bool
import cv2, numpy as np

try:
    import RPi.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

class AlertManager(Node):
    def __init__(self):
        super().__init__("alert_manager")
        self.get_logger().info("Omnisight Alert Manager starting...")
        self.config   = self._load_config()
        self.sim_mode = bool(self.config.get("simulation_mode", False))

        # Network config
        self.target_ip   = self.config.get("monitoring_device_ip",   "192.168.0.100")
        self.target_port = int(self.config.get("monitoring_device_port", 9999))
        self.timeout     = float(self.config.get("alert_socket_timeout_sec", 5.0))
        self.retries     = int(self.config.get("reconnect_attempts",  3))

        # Buzzer
        self.buzzer_pin  = int(self.config.get("buzzer_pin",  17))
        self.buzzer_dur  = float(self.config.get("buzzer_duration_sec", 3.0))

        # Log dir
        self.log_dir = self.config.get("log_dir", "/home/pi/omnisight_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        # TCP socket (persistent connection)
        self.sock         = None
        self._sock_lock   = threading.Lock()
        self._alert_queue = []
        self._queue_lock  = threading.Lock()

        # GPIO buzzer
        if HW_AVAILABLE and not self.sim_mode:
            self._init_buzzer()
        else:
            self.get_logger().info("Simulation mode — GPIO buzzer skipped")

        # Connect to monitoring device
        self._connect_socket()

        # ROS2
        self.create_subscription(String, "/omnisight/alert/stranger",     self._on_stranger, 10)
        self.create_subscription(String, "/omnisight/alert/scene_change", self._on_scene,    10)
        self.create_subscription(Bool,   "/omnisight/trigger_buzzer",     self._on_buzzer,   10)

        # Send queued alerts at 2 Hz
        self.create_timer(0.5, self._flush_queue)
        self.get_logger().info(
            f"Alert Manager ready → {self.target_ip}:{self.target_port}")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","patrol_waypoints.yaml"))
        try:
            with open(p) as f: return yaml.safe_load(f).get("omnisight",{})
        except: return {}

    def _init_buzzer(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.buzzer_pin, GPIO.OUT)
            GPIO.output(self.buzzer_pin, GPIO.LOW)
            self.get_logger().info(f"Buzzer ready on GPIO {self.buzzer_pin}")
        except Exception as e:
            self.get_logger().error(f"Buzzer init failed: {e}")

    def _buzz(self, duration=None):
        """Activate buzzer in background thread."""
        dur = duration or self.buzzer_dur
        def _do():
            try:
                if HW_AVAILABLE and not self.sim_mode:
                    GPIO.output(self.buzzer_pin, GPIO.HIGH)
                    time.sleep(dur)
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                else:
                    self.get_logger().info(f"  [SIM] BUZZER ON for {dur:.1f}s")
                    time.sleep(dur)
                    self.get_logger().info("  [SIM] BUZZER OFF")
            except Exception as e:
                self.get_logger().error(f"Buzzer error: {e}")
        threading.Thread(target=_do, daemon=True).start()

    def _connect_socket(self):
        """Establish persistent TCP socket to monitoring device."""
        for attempt in range(1, self.retries+1):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(self.timeout)
                s.connect((self.target_ip, self.target_port))
                with self._sock_lock: self.sock = s
                self.get_logger().info(
                    f"Connected to monitoring device {self.target_ip}:{self.target_port}")
                return
            except Exception as e:
                self.get_logger().warn(
                    f"Connection attempt {attempt}/{self.retries} failed: {e}")
                time.sleep(1.0)
        self.get_logger().warn(
            "Could not connect to monitoring device — alerts saved locally only")

    def _send_alert(self, payload_json: str):
        """
        Send alert over TCP socket.
        Protocol: 4-byte big-endian length + UTF-8 JSON bytes
        """
        data = payload_json.encode("utf-8")
        header = struct.pack(">I", len(data))
        with self._sock_lock:
            if self.sock is None:
                self._connect_socket()
            if self.sock:
                try:
                    self.sock.sendall(header + data)
                    self.get_logger().info(
                        f"Alert sent ({len(data)/1024:.1f} KB)")
                    return True
                except Exception as e:
                    self.get_logger().warn(f"Send failed: {e} — reconnecting...")
                    try: self.sock.close()
                    except: pass
                    self.sock = None
                    self._connect_socket()
        return False

    def _save_locally(self, alert_type: str, img_b64: str):
        """Save alert image to local log directory."""
        if not self.config.get("save_alert_images_locally", True): return
        try:
            ts   = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.log_dir, f"{alert_type}_{ts}.jpg")
            img_bytes = base64.b64decode(img_b64)
            with open(path, "wb") as f: f.write(img_bytes)
            self.get_logger().info(f"  Alert image saved: {path}")
        except Exception as e:
            self.get_logger().warn(f"Local save failed: {e}")

    def _on_stranger(self, msg: String):
        """Stranger detected alert from face_recognition_node."""
        self.get_logger().warn("=== STRANGER ALERT ===")
        self._buzz()  # immediately buzz
        try:
            data = json.loads(msg.data)
            self._save_locally("stranger", data.get("image_b64",""))
            with self._queue_lock:
                self._alert_queue.append(msg.data)
        except Exception as e:
            self.get_logger().error(f"Stranger alert parse error: {e}")

    def _on_scene(self, msg: String):
        """Scene change alert from scene_monitor."""
        try:
            data = json.loads(msg.data)
            self.get_logger().warn(
                f"=== SCENE CHANGE: {data.get('zone_label','?')} "
                f"(type={data.get('zone_type','?')}) ===")
            self._save_locally("scene_change", data.get("image_b64",""))
            with self._queue_lock:
                self._alert_queue.append(msg.data)
        except Exception as e:
            self.get_logger().error(f"Scene alert parse error: {e}")

    def _on_buzzer(self, msg: Bool):
        """Explicit buzzer trigger from face_recognition_node."""
        if msg.data: self._buzz()

    def _flush_queue(self):
        """Send all queued alerts at 2 Hz."""
        with self._queue_lock:
            queue_copy = list(self._alert_queue)
            self._alert_queue.clear()
        for payload in queue_copy:
            self._send_alert(payload)

    def destroy_node(self):
        with self._sock_lock:
            if self.sock:
                try: self.sock.close()
                except: pass
        if HW_AVAILABLE and not self.sim_mode:
            try: GPIO.cleanup()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlertManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

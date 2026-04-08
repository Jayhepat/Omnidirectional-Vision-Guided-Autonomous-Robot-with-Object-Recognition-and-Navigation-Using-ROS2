#!/usr/bin/env python3
"""
scene_monitor.py — Omnisight Scene Change Detection
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Captures baseline images of key zones on FIRST patrol.
On every subsequent patrol, compares current camera view
to baseline using SSIM (Structural Similarity Index).

If SSIM < threshold OR pixel diff > threshold:
  → Publishes /omnisight/alert/scene_change with image

Monitored zones (from room XML):
  - Entry door   (open/closed state)
  - Windows      (curtain/blind state)
  - Curtains     (displaced/moved)
  - Object areas (items removed/moved)
  - Room centre  (general scene change)
"""
import rclpy, os, yaml, cv2, time, json
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image
try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except: CV_BRIDGE = False
import numpy as np
import xml.etree.ElementTree as ET

class SceneMonitor(Node):
    def __init__(self):
        super().__init__("scene_monitor")
        self.get_logger().info("Omnisight Scene Monitor starting...")
        self.config      = self._load_config()
        self.zones       = self._load_zones()
        self.sim_mode    = bool(self.config.get("simulation_mode", False))
        self.baseline_dir= os.path.abspath(os.path.join(
            os.path.dirname(__file__),"..","config","baseline_images"))
        os.makedirs(self.baseline_dir, exist_ok=True)
        self.ssim_thresh = float(self.config.get("scene_ssim_threshold", 0.85))
        self.diff_thresh = float(self.config.get("scene_pixel_diff_threshold_pct", 15.0))
        self.patrol_count= 0
        self.baseline_cache = {}  # zone_id -> baseline image
        self.patrol_state   = "IDLE"
        self.cap = None
        self.bridge = CvBridge() if CV_BRIDGE else None

        # Load existing baselines from disk
        self._load_baselines_from_disk()

        # Publishers
        self.pub_alert = self.create_publisher(String, "/omnisight/alert/scene_change", 10)

        # Subscribers
        self.create_subscription(String, "/omnisight/patrol_state",   self._on_state,  10)
        self.create_subscription(Int32, "/omnisight/target_waypoint", self._on_wp, 10)

        # Camera
        if not self.sim_mode:
            cam_idx = int(self.config.get("camera_index", 0))
            self.cap = cv2.VideoCapture(cam_idx)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(self.config.get("camera_width",  640)))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.config.get("camera_height", 480)))
            self.get_logger().info(f"Camera opened at index {cam_idx}")

        # Scan at 1 Hz during patrol
        self.create_timer(1.0, self._scan_tick)
        self.get_logger().info(f"Scene monitor ready. {len(self.zones)} zones loaded. Baselines: {len(self.baseline_cache)}")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","patrol_waypoints.yaml"))
        try:
            with open(p) as f: return yaml.safe_load(f).get("omnisight",{})
        except: return {}

    def _load_zones(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","omnisight_room.xml"))
        zones = []
        try:
            root = ET.parse(p).getroot()
            for z in root.findall(".//zone"):
                g = lambda t: z.find(t).text if z.find(t) is not None else ""
                zones.append({"id":z.get("id","?"),"label":g("label"),"type":g("type"),
                    "near_wp":g("near_waypoint_id"),"threshold":float(g("change_threshold_pct") or 15.0)})
        except Exception as e:
            self.get_logger().error(f"Zone XML error: {e}")
        return zones

    def _load_baselines_from_disk(self):
        for zone in self.zones:
            path = os.path.join(self.baseline_dir, f"baseline_zone_{zone['id']}.jpg")
            if os.path.exists(path):
                img = cv2.imread(path)
                if img is not None:
                    self.baseline_cache[zone["id"]] = img
                    self.get_logger().info(f"  Loaded baseline: zone {zone['id']} ({zone['label']})")

    def _grab_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            return frame if ret else None
        # Simulation: return noise image
        return np.random.randint(100,200,(480,640,3),dtype=np.uint8)

    def _ssim(self, img1, img2):
        """Compute SSIM between two BGR images (simplified version)."""
        # Convert to grayscale
        g1 = cv2.cvtColor(cv2.resize(img1,(320,240)), cv2.COLOR_BGR2GRAY).astype(float)
        g2 = cv2.cvtColor(cv2.resize(img2,(320,240)), cv2.COLOR_BGR2GRAY).astype(float)
        mu1,mu2    = g1.mean(),g2.mean()
        sig1,sig2  = g1.std(),g2.std()
        sig12      = ((g1-mu1)*(g2-mu2)).mean()
        C1,C2      = (0.01*255)**2,(0.03*255)**2
        ssim_val   = ((2*mu1*mu2+C1)*(2*sig12+C2)) / ((mu1**2+mu2**2+C1)*(sig1**2+sig2**2+C2))
        return float(ssim_val)

    def _pixel_diff_pct(self, img1, img2):
        """Percentage of pixels that changed significantly."""
        diff   = cv2.absdiff(cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY),
                             cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY))
        thresh = 30  # pixel value change threshold
        changed= np.sum(diff > thresh)
        return changed / diff.size * 100.0

    def _capture_baseline(self, zone_id, label):
        """Capture and save baseline image for a zone."""
        frame = self._grab_frame()
        if frame is None: return
        self.baseline_cache[zone_id] = frame.copy()
        path = os.path.join(self.baseline_dir, f"baseline_zone_{zone_id}.jpg")
        cv2.imwrite(path, frame)
        self.get_logger().info(f"  Baseline captured: zone {zone_id} ({label})")

    def _check_zone(self, zone):
        """Compare current view to baseline for one zone."""
        zid = zone["id"]
        if zid not in self.baseline_cache:
            # No baseline yet — capture it
            self._capture_baseline(zid, zone["label"])
            return

        frame = self._grab_frame()
        if frame is None: return

        baseline = self.baseline_cache[zid]
        ssim_val  = self._ssim(baseline, frame)
        diff_pct  = self._pixel_diff_pct(baseline, frame)

        self.get_logger().debug(f"Zone {zid} [{zone['label']}]: SSIM={ssim_val:.3f}  diff={diff_pct:.1f}%")

        changed = (ssim_val < self.ssim_thresh) or (diff_pct > zone["threshold"])

        if changed:
            self.get_logger().warn(
                f"SCENE CHANGE DETECTED: Zone {zid} [{zone['label']}] "
                f"SSIM={ssim_val:.3f} diff={diff_pct:.1f}%")
            # Encode image and publish alert
            _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            payload = json.dumps({
                "type":       "scene_change",
                "zone_id":    str(zid),
                "zone_label": zone["label"],
                "zone_type":  zone["type"],
                "ssim":       round(ssim_val,3),
                "diff_pct":   round(diff_pct,1),
                "timestamp":  time.strftime("%Y-%m-%d %H:%M:%S"),
                "image_b64":  __import__("base64").b64encode(jpg.tobytes()).decode()
            })
            msg = String(); msg.data = payload
            self.pub_alert.publish(msg)

    def _scan_tick(self):
        """Called at 1 Hz. Check all zones if patrolling."""
        if self.patrol_state != "PATROLLING": return
        for zone in self.zones:
            self._check_zone(zone)

    def _on_state(self, msg):
        prev = self.patrol_state
        self.patrol_state = msg.data
        if msg.data == "PATROLLING" and prev != "PATROLLING":
            self.patrol_count += 1
            self.get_logger().info(f"Scene monitor: patrol cycle #{self.patrol_count}")

    def _on_wp(self, msg):
        """Capture fresh baselines at designated waypoints if first patrol."""
        if self.patrol_count == 1 and self.patrol_state == "PATROLLING":
            for zone in self.zones:
                if str(msg.data) == str(zone.get("near_wp","")):
                    self._capture_baseline(zone["id"], zone["label"])

    def destroy_node(self):
        if self.cap: self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SceneMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

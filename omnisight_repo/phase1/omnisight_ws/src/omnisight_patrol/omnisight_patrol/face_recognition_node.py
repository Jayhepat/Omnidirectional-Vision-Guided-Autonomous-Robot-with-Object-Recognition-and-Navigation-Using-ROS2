#!/usr/bin/env python3
"""
face_recognition_node.py — Omnisight Face Recognition & Stranger Detection
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Pipeline:
  1. MediaPipe BlazeFace detects face bounding boxes (~3ms)
  2. face_recognition library computes 128-D encoding (~18ms)
  3. Euclidean distance to known face database
  4. If min_distance > 0.55 threshold → STRANGER DETECTED
  5. Publish alert to /omnisight/alert/stranger
  6. Publish /omnisight/trigger_buzzer

Authorised faces: config/known_faces/name.jpg (one per person)
"""
import rclpy, os, yaml, cv2, json, time
from rclpy.node import Node
from std_msgs.msg import String, Bool
import numpy as np

try:
    import face_recognition as fr
    FR_AVAILABLE = True
except ImportError:
    FR_AVAILABLE = False

try:
    import mediapipe as mp
    MP_AVAILABLE = True
except ImportError:
    MP_AVAILABLE = False

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__("face_recognition_node")
        self.get_logger().info("Omnisight Face Recognition starting...")
        self.config   = self._load_config()
        self.sim_mode = bool(self.config.get("simulation_mode", False))
        self.threshold= float(self.config.get("face_distance_threshold", 0.55))
        self.patrol_state = "IDLE"
        self.cap = None

        # Load known faces
        self.known_encodings = []
        self.known_names     = []
        self._load_known_faces()

        # MediaPipe BlazeFace detector
        self.mp_face = None
        if MP_AVAILABLE:
            self.mp_face = mp.solutions.face_detection.FaceDetection(
                model_selection=0,
                min_detection_confidence=float(self.config.get("face_detection_confidence", 0.80)))
            self.get_logger().info("MediaPipe BlazeFace detector ready")

        # Camera
        if not self.sim_mode:
            cam_idx = int(self.config.get("camera_index", 0))
            self.cap = cv2.VideoCapture(cam_idx)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(self.config.get("camera_width",  640)))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.config.get("camera_height", 480)))

        # Publishers
        self.pub_stranger = self.create_publisher(String, "/omnisight/alert/stranger",    10)
        self.pub_buzzer   = self.create_publisher(Bool,   "/omnisight/trigger_buzzer",    10)

        # Subscribers
        self.create_subscription(String, "/omnisight/patrol_state", self._on_state, 10)

        # Process at 5 Hz (face recognition is slow ~21ms per face)
        self.create_timer(0.2, self._process_frame)
        self.get_logger().info(
            f"Face Recognition ready. Known faces: {len(self.known_names)} "
            f"[{', '.join(self.known_names)}]. Threshold: {self.threshold}")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","patrol_waypoints.yaml"))
        try:
            with open(p) as f: return yaml.safe_load(f).get("omnisight",{})
        except: return {}

    def _load_known_faces(self):
        faces_dir = os.path.abspath(os.path.join(
            os.path.dirname(__file__),"..","config","known_faces"))
        if not os.path.exists(faces_dir):
            self.get_logger().warn(f"Known faces dir not found: {faces_dir}")
            return
        loaded = 0
        for fname in os.listdir(faces_dir):
            if not fname.lower().endswith((".jpg",".jpeg",".png")): continue
            fpath = os.path.join(faces_dir, fname)
            try:
                img = fr.load_image_file(fpath) if FR_AVAILABLE else None
                if FR_AVAILABLE and img is not None:
                    encs = fr.face_encodings(img)
                    if encs:
                        self.known_encodings.append(encs[0])
                        name = os.path.splitext(fname)[0].replace("_"," ").title()
                        self.known_names.append(name)
                        loaded += 1
                        self.get_logger().info(f"  Loaded face: {name}")
            except Exception as e:
                self.get_logger().warn(f"  Failed to load {fname}: {e}")
        self.get_logger().info(f"Loaded {loaded} authorised faces")

    def _grab_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            return frame if ret else None
        return np.random.randint(50,200,(480,640,3),dtype=np.uint8)

    def _detect_with_mediapipe(self, frame_bgr):
        """Use MediaPipe BlazeFace to detect face bounding boxes."""
        if self.mp_face is None: return []
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results   = self.mp_face.process(frame_rgb)
        boxes = []
        if results.detections:
            h,w = frame_bgr.shape[:2]
            for det in results.detections:
                bb = det.location_data.relative_bounding_box
                x1 = max(0, int(bb.xmin * w))
                y1 = max(0, int(bb.ymin * h))
                x2 = min(w, int((bb.xmin+bb.width)  * w))
                y2 = min(h, int((bb.ymin+bb.height) * h))
                boxes.append((y1, x2, y2, x1))  # face_recognition format: top,right,bottom,left
        return boxes

    def _process_frame(self):
        """Main face recognition pipeline called at 5 Hz."""
        if self.patrol_state not in ("PATROLLING", "RETURNING"): return

        frame = self._grab_frame()
        if frame is None: return

        # Step 1: Detect faces with MediaPipe
        face_locations = self._detect_with_mediapipe(frame)
        if not face_locations: return

        # Step 2: Compute 128-D encodings
        if not FR_AVAILABLE:
            # Simulation: random stranger with 10% probability
            if np.random.rand() < 0.10:
                self._trigger_stranger_alert(frame, "SimulatedStranger", 0.72)
            return

        rgb_frame    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        encodings    = fr.face_encodings(rgb_frame, face_locations)

        for enc in encodings:
            name, dist = self._identify_face(enc)
            if name == "STRANGER":
                self.get_logger().warn(f"STRANGER DETECTED! Distance: {dist:.3f}")
                self._trigger_stranger_alert(frame, name, dist)
            else:
                self.get_logger().info(f"  Recognised: {name} (dist={dist:.3f})")

    def _identify_face(self, encoding):
        """Compare encoding to known faces. Return (name, distance)."""
        if not self.known_encodings:
            return "STRANGER", 1.0  # no known faces → everyone is stranger
        distances = fr.face_distance(self.known_encodings, encoding)
        min_idx   = int(np.argmin(distances))
        min_dist  = float(distances[min_idx])
        if min_dist <= self.threshold:
            return self.known_names[min_idx], min_dist
        return "STRANGER", min_dist

    def _trigger_stranger_alert(self, frame, name, dist):
        """Publish stranger alert with captured image."""
        _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        payload = json.dumps({
            "type":         "stranger_detected",
            "name":         name,
            "distance":     round(dist, 3),
            "threshold":    self.threshold,
            "timestamp":    time.strftime("%Y-%m-%d %H:%M:%S"),
            "image_b64":    __import__("base64").b64encode(jpg.tobytes()).decode()
        })
        msg = String(); msg.data = payload
        self.pub_stranger.publish(msg)
        # Trigger buzzer
        bz = Bool(); bz.data = True
        self.pub_buzzer.publish(bz)

    def _on_state(self, msg): self.patrol_state = msg.data

    def destroy_node(self):
        if self.cap: self.cap.release()
        if self.mp_face: self.mp_face.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

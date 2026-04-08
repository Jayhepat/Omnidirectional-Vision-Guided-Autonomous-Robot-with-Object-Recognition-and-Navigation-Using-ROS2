#!/usr/bin/env python3
"""
motion_control.py — Omnisight Mecanum Wheel Navigator
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Mecanum wheel inverse kinematics:
  FL =  vx - vy - omega*(Lx+Ly)
  FR =  vx + vy + omega*(Lx+Ly)
  RL =  vx + vy - omega*(Lx+Ly)
  RR =  vx - vy + omega*(Lx+Ly)

Subscribes : /omnisight/target_waypoint  (Int32)
Publishes  : /omnisight/waypoint_reached (Bool)
           : /omnisight/odom             (Odometry)
"""
import rclpy, os, yaml, math, time, threading
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
import xml.etree.ElementTree as ET

try:
    import busio
    from board import SCL, SDA
    from adafruit_pca9685 import PCA9685
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

class MotionController(Node):
    def __init__(self):
        super().__init__("motion_control")
        self.get_logger().info("Omnisight Motion Controller starting...")
        self.config    = self._load_config()
        self.waypoints = self._load_waypoints()
        self.speed     = float(self.config.get("patrol_speed_ms", 0.20))
        self.tolerance = float(self.config.get("waypoint_tolerance_m", 0.15) or 0.15)
        self.sim_mode  = bool(self.config.get("simulation_mode", False))
        self.pos_x     = float((self.config.get("base") or {}).get("x", 0.5))
        self.pos_y     = float((self.config.get("base") or {}).get("y", 0.5))
        self.heading   = 0.0
        self.moving    = False
        self.emergency_stop = False
        self.pca = None
        if HW_AVAILABLE and not self.sim_mode:
            self._init_motors()
        else:
            self.get_logger().info("Simulation mode or no hardware — motors skipped")

        self.pub_reached = self.create_publisher(Bool,     "/omnisight/waypoint_reached", 10)
        self.pub_odom    = self.create_publisher(Odometry, "/omnisight/odom",              10)
        self.create_subscription(Int32,  "/omnisight/target_waypoint",  self._on_target,   10)
        self.create_subscription(String, "/omnisight/patrol_state",     self._on_state,    10)
        self.create_subscription(Bool,   "/omnisight/obstacle_detected",self._on_obstacle, 10)
        self.create_subscription(Bool,   "/omnisight/path_clear",       self._on_clear,    10)
        self.create_timer(0.1, self._publish_odom)
        self.get_logger().info(f"Motion ready. Start pos ({self.pos_x:.2f},{self.pos_y:.2f}). Waypoints: {len(self.waypoints)}")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","patrol_waypoints.yaml"))
        try:
            with open(p) as f: return yaml.safe_load(f).get("omnisight", {})
        except: return {}

    def _load_waypoints(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","omnisight_room.xml"))
        wps = []
        try:
            root = ET.parse(p).getroot()
            for wp in root.findall(".//waypoint"):
                g = lambda t: wp.find(t).text if wp.find(t) is not None else "0"
                wps.append({"id":int(wp.get("id",0)),"x":float(g("x")),"y":float(g("y")),
                    "heading_deg":float(g("heading_deg")),"dwell_sec":float(g("dwell_sec")),"label":g("label")})
        except Exception as e:
            self.get_logger().error(f"Waypoint XML error: {e}")
            wps = [{"id":i,"x":x,"y":y,"heading_deg":0,"dwell_sec":1.5,"label":f"WP{i}"}
                   for i,(x,y) in enumerate([(0.5,0.5),(3,0.5),(5.5,0.5),(5.5,4.5),(0.5,4.5),(0.5,0.5)],1)]
        return wps

    def _init_motors(self):
        try:
            i2c = busio.I2C(SCL, SDA)
            addr = int(self.config.get("motor_i2c_address", 0x40))
            self.pca = PCA9685(i2c, address=addr)
            self.pca.frequency = int(self.config.get("motor_pwm_freq", 50))
            self.get_logger().info(f"PCA9685 motor driver ready at 0x{addr:02X}")
        except Exception as e:
            self.get_logger().error(f"Motor driver init failed: {e}"); self.pca = None

    def _mecanum_move(self, vx, vy, omega):
        Lxy = 0.27  # Lx+Ly
        w_fl =  vx - vy - omega*Lxy
        w_fr =  vx + vy + omega*Lxy
        w_rl =  vx + vy - omega*Lxy
        w_rr =  vx - vy + omega*Lxy
        max_w = max(abs(w_fl),abs(w_fr),abs(w_rl),abs(w_rr),1e-6)
        max_pwm = int(self.config.get("motor_max_pwm",4095))
        chs = [int(self.config.get(f"motor_{m}_channel",i)) for i,m in enumerate(["fl","fr","rl","rr"])]
        if self.pca:
            for ch,w in zip(chs,[w_fl,w_fr,w_rl,w_rr]):
                self.pca.channels[ch].duty_cycle = max(0,min(max_pwm,int(abs(w)/max_w*max_pwm)))

    def _stop_motors(self):
        if self.pca:
            for ch in range(4): self.pca.channels[ch].duty_cycle = 0
        self.moving = False

    def _navigate_to(self, tx, ty, dwell=1.0):
        self.moving = True
        self.get_logger().info(f"Navigating to ({tx:.2f},{ty:.2f})")
        dt = 0.05
        for _ in range(3000):
            if self.emergency_stop:
                self._stop_motors()
                while self.emergency_stop and rclpy.ok(): time.sleep(0.1)
            dx = tx-self.pos_x; dy = ty-self.pos_y
            dist = math.sqrt(dx*dx+dy*dy)
            if dist < self.tolerance: break
            vx_w = (dx/dist)*self.speed; vy_w = (dy/dist)*self.speed
            h_rad = math.radians(self.heading)
            vx_r =  vx_w*math.cos(h_rad)+vy_w*math.sin(h_rad)
            vy_r = -vx_w*math.sin(h_rad)+vy_w*math.cos(h_rad)
            t_hdg = math.degrees(math.atan2(dy,dx))
            omega = max(-0.5,min(0.5,((t_hdg-self.heading+180)%360-180)*0.02))
            self._mecanum_move(vx_r, vy_r, omega)
            self.pos_x += vx_w*dt; self.pos_y += vy_w*dt
            self.heading = (self.heading+math.degrees(omega)*dt)%360
            time.sleep(dt)
        self._stop_motors(); self.pos_x=tx; self.pos_y=ty
        if dwell>0: time.sleep(dwell)
        self.moving = False

    def _on_target(self, msg):
        idx = msg.data
        if idx >= 0:
            if idx < len(self.waypoints): wp=self.waypoints[idx]; tx,ty,dwell=wp["x"],wp["y"],wp.get("dwell_sec",1.0)
            else: return
        else:
            ret_idx = abs(idx)-1; ret_list = list(reversed(self.waypoints))
            if ret_idx < len(ret_list): wp=ret_list[ret_idx]; tx,ty,dwell=wp["x"],wp["y"],0.5
            else: return
        def nav_thread():
            self._navigate_to(tx,ty,dwell)
            m=Bool(); m.data=True; self.pub_reached.publish(m)
            self.get_logger().info(f"  Reached ({tx:.2f},{ty:.2f})")
        threading.Thread(target=nav_thread,daemon=True).start()

    def _on_state(self,msg):
        if msg.data=="EMERGENCY": self.emergency_stop=True; self._stop_motors()
    def _on_obstacle(self,msg):
        if msg.data: self.emergency_stop=True; self._stop_motors()
    def _on_clear(self,msg):
        if msg.data: self.emergency_stop=False
    def _publish_odom(self):
        m=Odometry(); m.header.stamp=self.get_clock().now().to_msg()
        m.header.frame_id="odom"; m.pose.pose.position.x=self.pos_x; m.pose.pose.position.y=self.pos_y
        self.pub_odom.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node=MotionController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node._stop_motors(); node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

#!/usr/bin/env python3
"""
patrol_master.py — Omnisight State Machine Brain
Omnisight | VIT Bhopal SEEE Capstone 2022-2026
States: IDLE → PATROLLING → RETURNING → WAITING → (repeat every 30 min)
"""
import rclpy, os, yaml, xml.etree.ElementTree as ET
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32

STATE_IDLE       = "IDLE"
STATE_PATROLLING = "PATROLLING"
STATE_RETURNING  = "RETURNING"
STATE_WAITING    = "WAITING"
STATE_EMERGENCY  = "EMERGENCY"

class PatrolMaster(Node):
    def __init__(self):
        super().__init__('patrol_master')
        self.get_logger().info("OMNISIGHT Patrol Master — VIT Bhopal SEEE 2022-2026")
        self.config    = self._load_config()
        self.waypoints = self._load_waypoints()
        self.state     = STATE_IDLE
        self.patrol_count       = 0
        self.current_wp         = 0
        self.obstacle_active    = False
        self.patrol_interval_s  = float(self.config.get('patrol_interval_minutes', 30)) * 60
        self.prev_state_before_emergency = STATE_PATROLLING

        self.pub_state   = self.create_publisher(String, '/omnisight/patrol_state',    10)
        self.pub_waypoint= self.create_publisher(Int32,  '/omnisight/target_waypoint', 10)
        self.pub_pantilt = self.create_publisher(String, '/omnisight/pantilt_preset',  10)

        self.create_subscription(Bool, '/omnisight/waypoint_reached', self._on_wp_reached, 10)
        self.create_subscription(Bool, '/omnisight/obstacle_detected', self._on_obstacle,  10)
        self.create_subscription(Bool, '/omnisight/path_clear',        self._on_clear,     10)

        self.create_timer(0.5, self._publish_state)
        self.start_timer = self.create_timer(5.0, self._start_first_patrol)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints. Starting patrol in 5s...")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),'..','config','patrol_waypoints.yaml'))
        try:
            with open(p) as f: return yaml.safe_load(f).get('omnisight',{})
        except Exception as e:
            self.get_logger().warn(f"Config load failed: {e}"); return {}

    def _load_waypoints(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),'..','config','omnisight_room.xml'))
        wps = []
        try:
            root = ET.parse(p).getroot()
            for wp in root.findall('.//waypoint'):
                g = lambda tag: wp.find(tag).text if wp.find(tag) is not None else '0'
                wps.append({'id':int(wp.get('id',0)),'x':float(g('x')),'y':float(g('y')),
                    'heading_deg':float(g('heading_deg')),'pan_angle':float(g('pan_angle_deg')),
                    'tilt_angle':float(g('tilt_angle_deg')),'dwell_sec':float(g('dwell_sec')),
                    'label':g('label')})
        except Exception as e:
            self.get_logger().error(f"XML load error: {e}")
            wps = [{'id':i,'x':x,'y':y,'heading_deg':0,'pan_angle':0,'tilt_angle':0,'dwell_sec':1.5,'label':f'WP{i}'}
                   for i,(x,y) in enumerate([(0.5,0.5),(3,0.5),(5.5,0.5),(5.5,4.5),(3,4.5),(0.5,4.5),(0.5,0.5)],1)]
        return wps

    def _start_first_patrol(self):
        self.start_timer.cancel()
        self.get_logger().info("=== OMNISIGHT: Starting Patrol Cycle #1 ===")
        self._enter_patrolling()

    def _enter_patrolling(self):
        self.state = STATE_PATROLLING; self.current_wp = 0; self.patrol_count += 1
        self.get_logger().info(f"STATE → PATROLLING  (Cycle #{self.patrol_count})")
        self._publish_state(); self._advance_waypoint()

    def _enter_returning(self):
        self.state = STATE_RETURNING
        self.get_logger().info("STATE → RETURNING  (Backtracking to Point A)")
        self._publish_state()
        self.return_waypoints = list(reversed(self.waypoints))
        self.current_return_wp = 0; self._advance_return_waypoint()

    def _enter_waiting(self):
        self.state = STATE_WAITING
        self.get_logger().info(f"STATE → WAITING  ({self.patrol_interval_s/60:.0f} min until next patrol)")
        self._publish_state()
        self.wait_timer = self.create_timer(self.patrol_interval_s, self._wait_complete)

    def _wait_complete(self):
        self.wait_timer.cancel()
        self.get_logger().info("=== Wait complete — starting next patrol ===")
        self._enter_patrolling()

    def _advance_waypoint(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info("Patrol complete → RETURNING"); self._enter_returning(); return
        wp = self.waypoints[self.current_wp]
        self.get_logger().info(f"  → WP {self.current_wp+1}/{len(self.waypoints)}: ({wp['x']:.1f},{wp['y']:.1f}) [{wp['label']}]")
        m = Int32(); m.data = self.current_wp; self.pub_waypoint.publish(m)
        p = String(); p.data = f"wp:{self.current_wp}:pan:{wp['pan_angle']:.0f}:tilt:{wp['tilt_angle']:.0f}"
        self.pub_pantilt.publish(p)

    def _advance_return_waypoint(self):
        if self.current_return_wp >= len(self.return_waypoints):
            self.get_logger().info("Docked at Point A → WAITING"); self._enter_waiting(); return
        wp = self.return_waypoints[self.current_return_wp]
        self.get_logger().info(f"  ← Return {self.current_return_wp+1}: ({wp['x']:.1f},{wp['y']:.1f})")
        m = Int32(); m.data = -(self.current_return_wp+1); self.pub_waypoint.publish(m)

    def _on_wp_reached(self, msg):
        if not msg.data: return
        if self.state == STATE_PATROLLING:
            self.current_wp += 1; self._advance_waypoint()
        elif self.state == STATE_RETURNING:
            self.current_return_wp += 1; self._advance_return_waypoint()

    def _on_obstacle(self, msg):
        if msg.data and self.state not in (STATE_EMERGENCY, STATE_WAITING, STATE_IDLE):
            self.prev_state_before_emergency = self.state
            self.state = STATE_EMERGENCY; self.obstacle_active = True
            self.get_logger().warn("STATE → EMERGENCY (obstacle)"); self._publish_state()

    def _on_clear(self, msg):
        if msg.data and self.state == STATE_EMERGENCY:
            self.state = self.prev_state_before_emergency; self.obstacle_active = False
            self.get_logger().info(f"Path clear → resuming {self.state}"); self._publish_state()

    def _publish_state(self):
        m = String(); m.data = self.state; self.pub_state.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

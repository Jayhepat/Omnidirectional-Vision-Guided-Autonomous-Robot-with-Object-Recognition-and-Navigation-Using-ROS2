#!/usr/bin/env python3
"""
obstacle_avoidance.py — Omnisight Ultrasonic Obstacle Avoidance
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Threshold-Based Reactive Algorithm (Module 2):
  u(t) = STOP  if d(t) <= T_stop  (0.30m)
  u(t) = SLOW  if d(t) <= T_warn  (0.50m)
  u(t) = GO    if d(t)  > T_warn

Publishes /omnisight/obstacle_detected (Bool)
         /omnisight/path_clear         (Bool)

Hardware: HC-SR04 ultrasonic sensor
  GPIO Trigger → 10us pulse → sensor fires
  GPIO Echo    → measures return time → distance = t*340/2
"""
import rclpy, os, yaml, time, statistics
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    import RPi.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Omnisight Obstacle Avoidance starting...")
        self.config     = self._load_config()
        self.sim_mode   = bool(self.config.get("simulation_mode", False))
        self.trig_pin   = int(self.config.get("ultrasonic_trigger_pin", 18))
        self.echo_pin   = int(self.config.get("ultrasonic_echo_pin",    24))
        self.T_stop     = float(self.config.get("obstacle_stop_distance_m",  0.30))
        self.T_warn     = float(self.config.get("obstacle_avoid_distance_m", 0.50))
        self.obstacle_active   = False
        self.consecutive_clear = 0
        self.patrol_state      = "IDLE"

        if HW_AVAILABLE and not self.sim_mode:
            self._init_sensor()
        else:
            self.get_logger().info("Simulation mode — ultrasonic sensor simulated")

        self.pub_obstacle = self.create_publisher(Bool,   "/omnisight/obstacle_detected", 10)
        self.pub_clear    = self.create_publisher(Bool,   "/omnisight/path_clear",         10)
        self.pub_dist     = self.create_publisher(Float32,"/omnisight/ultrasonic_distance",10)

        self.create_subscription(
            __import__("std_msgs.msg",fromlist=["String"]).String,
            "/omnisight/patrol_state", self._on_state, 10)

        # Measure at 10 Hz
        self.create_timer(0.1, self._measure_tick)
        self.get_logger().info(
            f"Obstacle avoidance ready. Stop:{self.T_stop}m Warn:{self.T_warn}m")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","patrol_waypoints.yaml"))
        try:
            with open(p) as f: return yaml.safe_load(f).get("omnisight",{})
        except: return {}

    def _init_sensor(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trig_pin, GPIO.LOW)
            time.sleep(0.1)
            self.get_logger().info(
                f"HC-SR04 ready. TRIG={self.trig_pin} ECHO={self.echo_pin}")
        except Exception as e:
            self.get_logger().error(f"Ultrasonic init failed: {e}")

    def _measure_distance(self):
        """Measure distance using HC-SR04. Returns metres."""
        if not HW_AVAILABLE or self.sim_mode:
            # Simulation: mostly clear, occasional obstacle
            import random
            return random.uniform(0.6, 2.5) if random.random() > 0.05 else 0.25

        samples = []
        for _ in range(5):
            try:
                GPIO.output(self.trig_pin, GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(self.trig_pin, GPIO.LOW)
                t_start = time.time()
                while GPIO.input(self.echo_pin) == 0:
                    if time.time()-t_start > 0.03: break
                pulse_start = time.time()
                while GPIO.input(self.echo_pin) == 1:
                    if time.time()-pulse_start > 0.03: break
                pulse_end = time.time()
                dist_m = (pulse_end - pulse_start) * 34300 / 2 / 100
                if 0.02 < dist_m < 4.0:
                    samples.append(dist_m)
            except: pass
            time.sleep(0.01)

        if not samples: return 4.0
        # Remove outliers using mean±2*std filter
        if len(samples) >= 3:
            mu  = statistics.mean(samples)
            std = statistics.stdev(samples) if len(samples)>1 else 0
            samples = [s for s in samples if abs(s-mu) <= 2*std]
        return statistics.mean(samples) if samples else 4.0

    def _measure_tick(self):
        """Called at 10 Hz — measure distance and publish state."""
        if self.patrol_state in ("IDLE", "WAITING"): return

        dist = self._measure_distance()

        # Publish distance
        d_msg = Float32(); d_msg.data = float(dist)
        self.pub_dist.publish(d_msg)

        if dist <= self.T_stop:
            if not self.obstacle_active:
                self.get_logger().warn(
                    f"OBSTACLE: {dist:.2f}m <= stop threshold {self.T_stop}m")
                self.obstacle_active   = True
                self.consecutive_clear = 0
                m = Bool(); m.data = True; self.pub_obstacle.publish(m)
        else:
            if self.obstacle_active:
                self.consecutive_clear += 1
                if self.consecutive_clear >= 5:  # 5 consecutive clear readings
                    self.obstacle_active   = False
                    self.consecutive_clear = 0
                    self.get_logger().info(f"Path clear: {dist:.2f}m")
                    m = Bool(); m.data = True; self.pub_clear.publish(m)

    def _on_state(self, msg): self.patrol_state = msg.data

    def destroy_node(self):
        if HW_AVAILABLE and not self.sim_mode:
            try: GPIO.cleanup()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

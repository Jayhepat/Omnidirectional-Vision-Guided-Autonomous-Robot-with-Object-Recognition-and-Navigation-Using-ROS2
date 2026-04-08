#!/usr/bin/env python3
"""
pan_tilt_scanner.py — Omnisight 2-DOF Pan-Tilt Camera Scanner
Omnisight | VIT Bhopal SEEE Capstone 2022-2026

Controls two servos to sweep the camera:
  - During PATROLLING: continuous left-right sweep + tilt variation
  - At each waypoint: moves to the preset angle defined in room XML
  - During RETURNING: forward-facing only
  - During EMERGENCY: pan left-right to monitor threat

Servo control via RPi GPIO PWM:
  Pulse width: 500us (0°) to 2500us (180°) at 50Hz
  Duty cycle  = pulse_us / 20000 * 100  (%)
"""
import rclpy, os, yaml, math, time, threading
from rclpy.node import Node
from std_msgs.msg import String, Bool

try:
    import RPi.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

class PanTiltScanner(Node):
    def __init__(self):
        super().__init__("pan_tilt_scanner")
        self.get_logger().info("Omnisight Pan-Tilt Scanner starting...")
        self.config   = self._load_config()
        self.sim_mode = bool(self.config.get("simulation_mode", False))

        self.pan_pin  = int(self.config.get("pan_servo_pin",  23))
        self.tilt_pin = int(self.config.get("tilt_servo_pin", 24))
        self.pan_min  = float(self.config.get("pan_min_deg",  -90))
        self.pan_max  = float(self.config.get("pan_max_deg",   90))
        self.tilt_min = float(self.config.get("tilt_min_deg", -30))
        self.tilt_max = float(self.config.get("tilt_max_deg",  45))
        self.sweep_spd= float(self.config.get("pan_sweep_speed_dps", 30))

        self.current_pan  = 0.0
        self.current_tilt = 0.0
        self.patrol_state = "IDLE"
        self.target_pan   = 0.0
        self.target_tilt  = 0.0
        self.sweep_active = False
        self._lock        = threading.Lock()

        # Hardware init
        self.pan_pwm  = None
        self.tilt_pwm = None
        if HW_AVAILABLE and not self.sim_mode:
            self._init_servos()
        else:
            self.get_logger().info("Simulation mode — servos not initialised")

        # ROS2
        self.create_subscription(String, "/omnisight/patrol_state",   self._on_state,      10)
        self.create_subscription(String, "/omnisight/pantilt_preset", self._on_preset,     10)
        self.create_subscription(Bool,   "/omnisight/obstacle_detected",self._on_obstacle, 10)

        # Servo update at 20 Hz
        self.create_timer(0.05, self._update_servos)
        # Sweep timer at 10 Hz
        self.sweep_thread = threading.Thread(target=self._sweep_loop, daemon=True)
        self.sweep_thread.start()

        self.get_logger().info("Pan-Tilt Scanner ready.")

    def _load_config(self):
        p = os.path.abspath(os.path.join(os.path.dirname(__file__),"..","config","patrol_waypoints.yaml"))
        try:
            with open(p) as f: return yaml.safe_load(f).get("omnisight",{})
        except: return {}

    def _init_servos(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pan_pin,  GPIO.OUT)
            GPIO.setup(self.tilt_pin, GPIO.OUT)
            self.pan_pwm  = GPIO.PWM(self.pan_pin,  50)
            self.tilt_pwm = GPIO.PWM(self.tilt_pin, 50)
            self.pan_pwm.start(7.5)
            self.tilt_pwm.start(7.5)
            self.get_logger().info(f"Servos ready on GPIO {self.pan_pin} (pan), {self.tilt_pin} (tilt)")
        except Exception as e:
            self.get_logger().error(f"Servo init failed: {e}")

    def _angle_to_duty(self, angle_deg):
        """Convert angle (-90 to +90) to PWM duty cycle (2.5% to 12.5%)."""
        # Map [-90,90] → [500us,2500us] → [2.5%,12.5%] at 50Hz
        pulse_us = 1500 + (angle_deg / 90.0) * 1000
        pulse_us = max(500, min(2500, pulse_us))
        return pulse_us / 20000 * 100

    def _set_pan(self, angle_deg):
        angle_deg = max(self.pan_min, min(self.pan_max, angle_deg))
        self.current_pan = angle_deg
        if self.pan_pwm:
            self.pan_pwm.ChangeDutyCycle(self._angle_to_duty(angle_deg))

    def _set_tilt(self, angle_deg):
        angle_deg = max(self.tilt_min, min(self.tilt_max, angle_deg))
        self.current_tilt = angle_deg
        if self.tilt_pwm:
            self.tilt_pwm.ChangeDutyCycle(self._angle_to_duty(angle_deg))

    def _sweep_loop(self):
        """
        Continuous sweep during PATROLLING.
        Pattern: slow left-right pan with gentle tilt variation.
        """
        t = 0.0
        while rclpy.ok():
            if self.patrol_state == "PATROLLING" and self.sweep_active:
                # Sinusoidal pan sweep covering 80% of range
                pan_amp   = (self.pan_max - self.pan_min) * 0.4
                pan_angle = pan_amp * math.sin(t * 0.3)   # period ~21s
                tilt_angle= 10.0 * math.sin(t * 0.15)    # gentle tilt bob
                with self._lock:
                    self.target_pan  = pan_angle
                    self.target_tilt = tilt_angle
                t += 0.1
            elif self.patrol_state == "RETURNING":
                with self._lock:
                    self.target_pan  = 0.0
                    self.target_tilt = 0.0
            time.sleep(0.1)

    def _update_servos(self):
        """Smoothly move servos toward target angles at 20Hz."""
        with self._lock:
            tp = self.target_pan
            tt = self.target_tilt
        step = self.sweep_spd * 0.05  # degrees per 50ms tick
        new_pan  = self.current_pan  + max(-step, min(step, tp - self.current_pan))
        new_tilt = self.current_tilt + max(-step, min(step, tt - self.current_tilt))
        self._set_pan(new_pan)
        self._set_tilt(new_tilt)

    def _on_state(self, msg):
        self.patrol_state = msg.data
        if msg.data == "PATROLLING":
            self.sweep_active = True
        elif msg.data in ("RETURNING", "WAITING", "IDLE"):
            self.sweep_active = False
            with self._lock: self.target_pan=0.0; self.target_tilt=0.0
        elif msg.data == "EMERGENCY":
            # Rapidly pan to find obstacle direction
            self.sweep_active = False
            with self._lock: self.target_pan=0.0; self.target_tilt=10.0

    def _on_preset(self, msg):
        """Parse preset string: 'wp:N:pan:X:tilt:Y'"""
        try:
            parts = msg.data.split(":")
            pan_val  = float(parts[parts.index("pan")+1])
            tilt_val = float(parts[parts.index("tilt")+1])
            with self._lock:
                self.target_pan  = pan_val
                self.target_tilt = tilt_val
                self.sweep_active= False  # go to preset first, then resume sweep
            # Resume sweep after reaching preset
            def resume():
                time.sleep(2.0)
                self.sweep_active = True
            threading.Thread(target=resume, daemon=True).start()
        except Exception as e:
            self.get_logger().warn(f"Preset parse error: {e}")

    def _on_obstacle(self, msg):
        if msg.data:
            with self._lock: self.target_pan=0.0; self.target_tilt=15.0
            self.sweep_active = False

    def destroy_node(self):
        if self.pan_pwm:  self.pan_pwm.stop()
        if self.tilt_pwm: self.tilt_pwm.stop()
        if HW_AVAILABLE:  GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltScanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

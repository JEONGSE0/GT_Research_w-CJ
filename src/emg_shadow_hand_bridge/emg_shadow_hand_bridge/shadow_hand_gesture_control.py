#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, math, select, termios, tty, fcntl, atexit, signal, threading, queue, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# ---------------- Names & utils ----------------
JOINT_NAMES = [
    "robot0_WRJ1","robot0_WRJ0",
    "robot0_FFJ3","robot0_FFJ2","robot0_FFJ1","robot0_FFJ0",
    "robot0_MFJ3","robot0_MFJ2","robot0_MFJ1","robot0_MFJ0",
    "robot0_RFJ3","robot0_RFJ2","robot0_RFJ1","robot0_RFJ0",
    "robot0_LFJ4","robot0_LFJ3","robot0_LFJ2","robot0_LFJ1","robot0_LFJ0",
    "robot0_THJ4","robot0_THJ3","robot0_THJ2","robot0_THJ1","robot0_THJ0",
]
N = len(JOINT_NAMES)
d2r = math.radians

# Joint limits (rad)
LOW  = np.array([d2r(-28.018), d2r(-45.0),
                 d2r(-19.996),0,0,0,  d2r(-19.996),0,0,0,  d2r(-19.996),0,0,0,
                 0,d2r(-19.996),0,0,0,  d2r(-59.989),0,d2r(-11.975),d2r(-30.023),d2r(-90.012)], np.float32)
HIGH = np.array([d2r(20.0),   d2r(40.0),
                 d2r(19.996),d2r(90.012),d2r(90.012),d2r(90.012),
                 d2r(19.996),d2r(90.012),d2r(90.012),d2r(90.012),
                 d2r(19.996),d2r(90.012),d2r(90.012),d2r(90.012),
                 d2r(44.977),d2r(19.996),d2r(90.012),d2r(90.012),d2r(90.012),
                 d2r(59.989),d2r(70.015),d2r(11.975),d2r(30.023),0.0], np.float32)

def clamp(q): return np.minimum(np.maximum(q, LOW), HIGH)

# Poses (rad)
DEFAULT_POSE = np.zeros(N, np.float32)

FIST_POSE = clamp(np.array([
    d2r(-0.1), d2r(11.4),
    d2r(0.0), d2r(90.0), d2r(90.0), d2r(59.1),
    d2r(0.0), d2r(90.0), d2r(90.0), d2r(57.4),
    d2r(0.0), d2r(90.0), d2r(90.0), d2r(58.6),
    d2r(0.0), d2r(0.0),  d2r(90.0), d2r(90.0), d2r(60.3),
    d2r(27.1), d2r(10.0), d2r(0.0), d2r(-18.0), d2r(-75.0)
], np.float32))

# Wrist-only poses (others = 0)
ULNAR_POSE  = clamp(np.array([d2r(-28.019)] + [0.0]*(N-1), np.float32))   # WRJ1
RADIAL_POSE = clamp(np.array([d2r(20.0)]    + [0.0]*(N-1), np.float32))    # WRJ1
EXT_POSE    = clamp(np.array([0.0, d2r(-45.0)] + [0.0]*(N-2), np.float32)) # WRJ0
FLEX_POSE   = clamp(np.array([0.0, d2r(40.0)]  + [0.0]*(N-2), np.float32)) # WRJ0

# Abduction (others = 0)
ABDUCTION_POSE = np.zeros(N, np.float32)
ABDUCTION_POSE[2]  = d2r(20.0)    # FFJ3
ABDUCTION_POSE[6]  = d2r(8.0)     # MFJ3
ABDUCTION_POSE[10] = d2r(-8.0)    # RFJ3
ABDUCTION_POSE[15] = d2r(-20.0)   # LFJ3
ABDUCTION_POSE[19] = d2r(-26.0)   # THJ4
ABDUCTION_POSE[20] = d2r(18.0)    # THJ3
ABDUCTION_POSE = clamp(ABDUCTION_POSE)

# Pinch (기존 FINCH_POSE 활용)
PINCH_POSE = clamp(np.array([
    d2r(0.0), d2r(0.0),
    d2r(-13.8), d2r(61.9), d2r(43.8), d2r(10.4),
    d2r(-8.4), d2r(67.1), d2r(53.7), d2r(8.7),
    d2r(0.0), d2r(90.0), d2r(90.0), d2r(58.6),
    d2r(0.0), d2r(0.0),  d2r(90.0), d2r(90.0), d2r(60.3),
    d2r(20.3), d2r(70.0), d2r(-5.9), d2r(-17.0), d2r(-9.0)
], np.float32))

# ---------------- Safe terminal helper ----------------
class SafeTerminal:
    """Puts stdin into cbreak + nonblocking and restores it on exit."""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.is_tty = os.isatty(self.fd)
        self.saved_attr = None
        self.saved_fl = None
    def __enter__(self):
        if not self.is_tty: return self
        self.saved_attr = termios.tcgetattr(self.fd)
        self.saved_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        tty.setcbreak(self.fd)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.saved_fl | os.O_NONBLOCK)
        return self
    def __exit__(self, *_):
        self.restore()
    def restore(self):
        if not self.is_tty: return
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved_attr)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.saved_fl)
        except Exception:
            pass

# ---------------- Nonblocking key reader ----------------
class KeyReader(threading.Thread):
    def __init__(self, out_q, poll_hz=300):
        super().__init__(daemon=True)
        self.q = out_q
        self.dt = 1.0/poll_hz
        self.stop_flag = threading.Event()
        self.st = SafeTerminal()
    def run(self):
        with self.st:
            if not self.st.is_tty:
                # fallback (needs ENTER)
                while not self.stop_flag.is_set():
                    r,_,_ = select.select([sys.stdin], [], [], 0.1)
                    if r:
                        ch = sys.stdin.read(1)
                        self.q.put(ch)
                return
            # TTY cbreak + nonblocking
            while not self.stop_flag.is_set():
                r,_,_ = select.select([sys.stdin], [], [], self.dt)
                if r:
                    try:
                        ch = os.read(self.st.fd, 1).decode(errors="ignore")
                        if ch:
                            self.q.put(ch)
                    except BlockingIOError:
                        pass
    def stop(self):
        self.stop_flag.set()
        self.st.restore()

# ---------------- ROS node ----------------
class HandNode(Node):
    def __init__(self):
        super().__init__("shadow_hand_control")

        # ---- parameters ----
        self.declare_parameter("topic", "joint_command")
        self.declare_parameter("rate_hz", 100.0)       # motion tick
        self.declare_parameter("vmax_pose", 0.6)       # rad/s (rate limit)
        self.declare_parameter("gesture_topic", "/emg/gesture")
        self.declare_parameter("debounce_sec", 0.10)   # EMG 명령 간 최소 간격

        self.topic  = self.get_parameter("topic").get_parameter_value().string_value
        self.dt     = 0.1/float(self.get_parameter("rate_hz").value)
        self.vpose  = float(self.get_parameter("vmax_pose").value)
        self.gtopic = self.get_parameter("gesture_topic").get_parameter_value().string_value
        self.debounce = float(self.get_parameter("debounce_sec").value)

        # ---- pubs/subs ----
        self.pub = self.create_publisher(JointState, self.topic, 10)
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub_g = self.create_subscription(String, self.gtopic, self._on_gesture, qos)

        # ---- state ----
        self.msg = JointState(); self.msg.name = JOINT_NAMES
        self.q_cmd = DEFAULT_POSE.copy()
        self.pose_target = DEFAULT_POSE.copy()
        self.seq = "idle"
        self.enabled = False              # ← 1로 시작, 2로 중지
        self._last_g_time = 0.0
        self._last_g_label = None

        # ---- timer ----
        self.timer = self.create_timer(self.dt, self._tick)

        # ---- keyboard ----
        self.keyq = queue.Queue()
        self.kr = KeyReader(self.keyq, poll_hz=300); self.kr.start()
        atexit.register(self.kr.stop)
        signal.signal(signal.SIGINT,  lambda *_: self._shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self._shutdown())

        # 안내
        self.get_logger().info(
            "Controls:\n"
            " [1]= START EMG control | [2]= STOP (freeze)\n"
            " Manual pose: 3 Ulnar, 4 Radial, 5 Ext, 6 Flex, 7 Abduction, 8 Pinch, 0 Rest, 9 Grip\n"
            " q=Quit\n"
            f" Subscribing gestures from: {self.gtopic}"
        )

        # 제스처→포즈 매핑 (소문자 라벨 기준)
        self.G2POSE = {
            "right": EXT_POSE,           # extension
            "left": FLEX_POSE,           # flexion
            "up": RADIAL_POSE,           # radial deviation
            "down": ULNAR_POSE,          # ulnar deviation
            "close": FIST_POSE,          # grip
            "open": ABDUCTION_POSE,      # abduction
            "comeback": PINCH_POSE,      # pinch
            "rest": DEFAULT_POSE,
        }

    # ---------- helpers ----------
    def _shutdown(self):
        try: self.kr.stop()
        finally: rclpy.shutdown()

    @staticmethod
    def ramp(curr, target, vmax, dt):
        delta = target - curr
        step  = np.clip(delta, -vmax*dt, vmax*dt)
        return curr + step

    def _start_pose(self, pose: np.ndarray, reason=""):
        self.pose_target = clamp(pose.copy())
        self.seq = "to_pose"
        if reason:
            self.get_logger().info(reason)

    # ---------- inputs ----------
    def _handle_keys(self):
        while not self.keyq.empty():
            k = self.keyq.get_nowait()
            if   k == '1':
                self.enabled = True
                self.get_logger().info("EMG control: ENABLED")
            elif k == '2':
                self.enabled = False
                self.get_logger().info("EMG control: DISABLED (hold current)")
            elif k == '0':
                self._start_pose(DEFAULT_POSE, "→ Rest")
            elif k == '9':
                self._start_pose(FIST_POSE, "→ Grip (one-shot)")
            elif k == '3':
                self._start_pose(ULNAR_POSE, "→ Ulnar deviation")
            elif k == '4':
                self._start_pose(RADIAL_POSE, "→ Radial deviation")
            elif k == '5':
                self._start_pose(EXT_POSE, "→ Wrist extension")
            elif k == '6':
                self._start_pose(FLEX_POSE, "→ Wrist flexion")
            elif k == '7':
                self._start_pose(ABDUCTION_POSE, "→ Abduction")
            elif k == '8':
                self._start_pose(PINCH_POSE, "→ Pinch")
            elif k == 'q':
                self._shutdown()

    def _on_gesture(self, msg: String):
        if not self.enabled:
            return
        now = time.monotonic()
        if now - self._last_g_time < self.debounce:
            return

        # 메시지 형식 "Label (0.95)" → 괄호 전까지
        raw = msg.data.strip()
        label = raw.split('(')[0].strip().lower()

        if label in self.G2POSE:
            if label != self._last_g_label:
                self._last_g_label = label
                self._start_pose(self.G2POSE[label], f"[EMG] → {label}")
            self._last_g_time = now

    # ---------- main loop ----------
    def _tick(self):
        self._handle_keys()

        if self.seq == "to_pose":
            self.q_cmd = self.ramp(self.q_cmd, self.pose_target, self.vpose, self.dt)
            if np.allclose(self.q_cmd, self.pose_target, atol=1e-3):
                self.seq = "idle"

        # clamp & publish
        self.q_cmd = clamp(self.q_cmd)
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = self.q_cmd.tolist()
        self.msg.velocity = []; self.msg.effort = []
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = HandNode()
    try:
        rclpy.spin(node)
    finally:
        node.kr.stop()
        node.destroy_node()

if __name__ == "__main__":
    main()

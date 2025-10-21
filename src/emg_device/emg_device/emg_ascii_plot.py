#!/usr/bin/env python3
import os
import time
import threading
from typing import List, Optional
from collections import deque   # ★ 추가

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from ascii_graph import Pyasciigraph
    HAS_ASCII_GRAPH = True
except Exception:
    HAS_ASCII_GRAPH = False

class EmgAsciiPlot(Node):
    def __init__(self):
        super().__init__('emg_ascii_plot')

        # ── parameters ──────────────────────────────────────────────
        self.declare_parameter('topic', '/emg/signal')
        self.declare_parameter('print_rate_hz', 10.0)
        self.declare_parameter('scale_mode', 'fixed')
        self.declare_parameter('max_abs_uV', 2000.0)
        self.declare_parameter('title', 'EMG values (μV)')
        self.declare_parameter('clear_cmd', True)

        self.topic = self.get_parameter('topic').value
        self.print_rate_hz = float(self.get_parameter('print_rate_hz').value)
        self.scale_mode = str(self.get_parameter('scale_mode').value).lower()
        self.max_abs_uV = float(self.get_parameter('max_abs_uV').value)
        self.title = str(self.get_parameter('title').value)
        self.clear_cmd = bool(self.get_parameter('clear_cmd').value)

        # 최신 샘플 보관
        self._last_sample: Optional[List[float]] = None
        self._lock = threading.Lock()

        # ★ 수신 주파수 계산용 타임스탬프 버퍼(최대 200개)
        self._rx_times = deque(maxlen=200)

        # 구독자
        self.sub = self.create_subscription(
            Float32MultiArray, self.topic, self._cb, 10
        )

        # 프린트 타이머
        period = 1.0 / max(1e-6, self.print_rate_hz)
        self.timer = self.create_timer(period, self._print_once)

        # ascii_graph 준비
        if HAS_ASCII_GRAPH:
            self.graph = Pyasciigraph(multivalue=False)
            self.get_logger().info('Using ascii_graph for plotting.')
        else:
            self.graph = None
            self.get_logger().warn('ascii_graph not found. Using built-in fallback bars.\n'
                                   'Install with:  pip install ascii_graph')

        self.get_logger().info(f"Subscribing: {self.topic}")
        self.get_logger().info(f"print_rate_hz={self.print_rate_hz}, "
                               f"scale_mode={self.scale_mode}, max_abs_uV={self.max_abs_uV}")

    # ── 콜백: 최신 샘플 저장 ──────────────────────────────────────
    def _cb(self, msg: Float32MultiArray):
        with self._lock:
            self._last_sample = list(msg.data)
            # ★ 수신 시각 저장(ROS Clock 기준, 초 단위 float)
            now = self.get_clock().now()
            self._rx_times.append(now.nanoseconds * 1e-9)

    # ── 한 번 출력 ────────────────────────────────────────────────
    def _print_once(self):
        with self._lock:
            sample = self._last_sample
            rx_times = list(self._rx_times)
        if sample is None:
            return

        # ★ 수신 Hz 추정 (이동창 평균)
        rx_hz = 0.0
        last_dt_ms = None
        if len(rx_times) >= 2:
            dt = rx_times[-1] - rx_times[0]
            if dt > 0:
                rx_hz = (len(rx_times) - 1) / dt
            # 최근 간격(ms)도 참고용 출력
            last_dt_ms = (rx_times[-1] - rx_times[-2]) * 1000.0

        # 8채널로 잘라서 사용
        vals = sample[:8]
        # 스케일 결정
        if self.scale_mode == 'auto':
            max_abs = max(1.0, max(abs(v) for v in vals))
        else:
            max_abs = max(1.0, self.max_abs_uV)

        # 화면 정리
        if self.clear_cmd:
            os.system('clear')

        # 헤더 (★ 수신 Hz / 출력 Hz 표기)
        hdr = f'[{self.title}]  topic: {self.topic}   (scale ±{max_abs:.0f} μV)'
        rate_info = f' | rx_hz≈{rx_hz:6.2f}  print_hz={self.print_rate_hz:g}'
        if last_dt_ms is not None:
            rate_info += f'  (last Δ={last_dt_ms:.1f} ms)'
        print(hdr + rate_info)
        print('─' * 70)

        if HAS_ASCII_GRAPH and self.graph is not None:
            data = [('MAX', max_abs)]
            for i, v in enumerate(vals, start=1):
                data.append((f'CH{i} ({int(v):d})', abs(v)))
            for line in self.graph.graph('', data):
                print(line)
        else:
            width = 50
            for i, v in enumerate(vals, start=1):
                frac = min(1.0, abs(v) / max_abs)
                nbar = int(frac * width)
                bar = '█' * nbar + ' ' * (width - nbar)
                sign = '-' if v < 0 else ' '
                print(f'CH{i:02d} {sign} |{bar}| {v:8.0f} μV')
        print()

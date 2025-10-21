#!/usr/bin/env python3
import os
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# (옵션) ascii_graph 있으면 사용, 없으면 fallback 사용
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
        self.declare_parameter('print_rate_hz', 10.0)       # 터미널 업데이트 속도
        self.declare_parameter('scale_mode', 'fixed')       # 'fixed' or 'auto'
        self.declare_parameter('max_abs_uV', 2000.0)        # fixed일 때 기준(μV)
        self.declare_parameter('title', 'EMG values (μV)')
        self.declare_parameter('clear_cmd', True)           # 터미널 clear 사용 여부

        self.topic = self.get_parameter('topic').value
        self.print_rate_hz = float(self.get_parameter('print_rate_hz').value)
        self.scale_mode = str(self.get_parameter('scale_mode').value).lower()
        self.max_abs_uV = float(self.get_parameter('max_abs_uV').value)
        self.title = str(self.get_parameter('title').value)
        self.clear_cmd = bool(self.get_parameter('clear_cmd').value)

        # 최신 샘플 보관
        self._last_sample: Optional[List[float]] = None
        self._lock = threading.Lock()

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
        # msg.data 길이는 8을 기대(그 이상/이하여도 최소 8개까지만 표시)
        with self._lock:
            self._last_sample = list(msg.data)

    # ── 한 번 출력 ────────────────────────────────────────────────
    def _print_once(self):
        with self._lock:
            sample = self._last_sample
        if sample is None:
            return

        # 8채널로 잘라서 사용
        vals = sample[:8]
        # 스케일 결정
        if self.scale_mode == 'auto':
            max_abs = max(1.0, max(abs(v) for v in vals))
        else:
            max_abs = max(1.0, self.max_abs_uV)

        # 화면 정리
        if self.clear_cmd:
            # 더 빠른 ANSI clear: print("\033[2J\033[H", end="")
            os.system('clear')

        # 헤더
        print(f'[{self.title}]  topic: {self.topic}   (scale ±{max_abs:.0f} μV)')
        print('─' * 70)

        if HAS_ASCII_GRAPH and self.graph is not None:
            # ascii_graph 사용
            data = [('MAX', max_abs)]  # 스케일 막대
            for i, v in enumerate(vals, start=1):
                data.append((f'CH{i} ({int(v):d})', abs(v)))
            for line in self.graph.graph('', data):
                print(line)
        else:
            # fallback 간단 바(유니코드)
            width = 50
            for i, v in enumerate(vals, start=1):
                frac = min(1.0, abs(v) / max_abs)
                nbar = int(frac * width)
                bar = '█' * nbar + ' ' * (width - nbar)
                sign = '-' if v < 0 else ' '
                print(f'CH{i:02d} {sign} |{bar}| {v:8.0f} μV')
        print()

def main():
    rclpy.init()
    node = EmgAsciiPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

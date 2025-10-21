#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EMG 실시간 분류 노드 (디버그 강화판)

입력:  /emg/signal (std_msgs/Float32MultiArray) 8채널 단일 샘플
전처리: 실시간 필터(60/120Hz 노치 + 20~100Hz 밴드패스) -> 슬라이딩 윈도우(8 x window_size)
모델:  EMG_CNN (가중치 구조 불일치 시 fc 계층은 부분 로드로 fallback)
출력:  /emg/gesture (std_msgs/String: "Label (0.95)")
      /emg/gesture_idx (std_msgs/Int32)
      /emg/gesture_confidence (std_msgs/Float32)
옵션:  /emg/gesture_probs (Float32MultiArray)  # publish_probs=True 일 때
디버그:
  - debug=0(기본): 최소 로그
  - debug=1: 콜백 수신/윈도우 충족/추론/퍼블리시 이벤트 로그
  - debug=2: 1초 단위 상태 요약(수신 rate, 버퍼 길이, 추론/예외 카운트 등)
"""

from typing import List
from collections import deque
import os
import math
import time

import numpy as np
import torch
import torch.nn.functional as F

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Float32MultiArray, String, Int32, Float32
from ament_index_python.packages import get_package_share_directory

from emg_device.real_time_filter import RealtimeFilter
from emg_device.model_emg import EMG_CNN


class EmgClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__("emg_classifier_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("class_names", ["Rest","Close","Open","right","comeback","left","down","up"])
        self.declare_parameter("window_size", 250)
        self.declare_parameter("num_channels", 8)
        self.declare_parameter("fs", 250)
        self.declare_parameter("model_path", "")
        self.declare_parameter("input_topic", "/emg/signal")

        self.declare_parameter("gesture_topic", "/emg/gesture")
        self.declare_parameter("gesture_idx_topic", "/emg/gesture_idx")
        self.declare_parameter("gesture_conf_topic", "/emg/gesture_confidence")
        self.declare_parameter("publish_probs", False)
        self.declare_parameter("gesture_probs_topic", "/emg/gesture_probs")

        self.declare_parameter("qos_depth", 50)
        self.declare_parameter("device", "auto")   # cpu|cuda|auto
        self.declare_parameter("debug", 1)         # 0=min, 1=event, 2=1Hz summary

        # -------------- Read params --------------
        self.class_names: List[str] = list(
            self.get_parameter("class_names").get_parameter_value().string_array_value
        )
        self.window_size: int = int(self.get_parameter("window_size").get_parameter_value().integer_value)
        self.num_channels: int = int(self.get_parameter("num_channels").get_parameter_value().integer_value)
        self.fs: int = int(self.get_parameter("fs").get_parameter_value().integer_value)
        model_path_param: str = self.get_parameter("model_path").get_parameter_value().string_value

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.gesture_topic = self.get_parameter("gesture_topic").get_parameter_value().string_value
        self.gesture_idx_topic = self.get_parameter("gesture_idx_topic").get_parameter_value().string_value
        self.gesture_conf_topic = self.get_parameter("gesture_conf_topic").get_parameter_value().string_value
        self.publish_probs = self.get_parameter("publish_probs").get_parameter_value().bool_value
        self.gesture_probs_topic = self.get_parameter("gesture_probs_topic").get_parameter_value().string_value

        qos_depth = int(self.get_parameter("qos_depth").get_parameter_value().integer_value)
        self.debug = int(self.get_parameter("debug").get_parameter_value().integer_value)

        # -------------- Device --------------
        device_pref = (self.get_parameter("device").get_parameter_value().string_value or "auto").lower()
        if device_pref == "cpu":
            device = torch.device("cpu")
        elif device_pref == "cuda":
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._device = device

        # -------------- QoS --------------
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # -------------- Filter / Buffers --------------
        self.filter = RealtimeFilter(fs=self.fs)
        self.buffers = [deque(maxlen=self.window_size) for _ in range(self.num_channels)]

        # -------------- Model path --------------
        if model_path_param:
            model_path = model_path_param
        else:
            try:
                pkg_share = get_package_share_directory("emg_device")
                model_path = os.path.join(pkg_share, "models", "emg_cnn_model.pt")
            except Exception as e:
                self.get_logger().warn(f"[init] package share locate fail: {e}")
                model_path = "emg_cnn_model.pt"

        # -------------- Model load (strict -> partial) --------------
        self.model = EMG_CNN(num_classes=len(self.class_names))
        try:
            state = torch.load(model_path, map_location=device)
            try:
                self.model.load_state_dict(state)  # strict
                strict_loaded = True
            except Exception as e:
                strict_loaded = False
                self.get_logger().warn(f"[init] strict load failed: {e}; try partial (drop fc*)")
                filtered = {k: v for k, v in state.items() if not (k.startswith("fc1.") or k.startswith("fc2."))}
                missing, unexpected = self.model.load_state_dict(filtered, strict=False)
                self.get_logger().info(f"[init] partial load ok. missing={missing}, unexpected={unexpected}")
            self.model.to(device)
            self.model.eval()
            self.get_logger().info(
                f"[init] model loaded: {model_path} | strict={strict_loaded} | device={device}"
            )
        except Exception as e:
            self.get_logger().fatal(f"[init] model load failed: {e}")
            raise

        # -------------- ROS IO --------------
        self.sub = self.create_subscription(Float32MultiArray, self.input_topic, self._on_emg, qos)
        self.pub_gesture = self.create_publisher(String, self.gesture_topic, qos)
        self.pub_gesture_idx = self.create_publisher(Int32, self.gesture_idx_topic, qos)
        self.pub_gesture_conf = self.create_publisher(Float32, self.gesture_conf_topic, qos)
        self.pub_gesture_probs = (
            self.create_publisher(Float32MultiArray, self.gesture_probs_topic, qos) if self.publish_probs else None
        )

        # -------------- Runtime counters (for diagnostics) --------------
        self._recv_count = 0
        self._infer_ok = 0
        self._infer_exc = 0
        self._pub_count = 0
        self._last_summary = time.time()
        self._last_cb_time = None

        # 1Hz 상태 요약 타이머
        if self.debug >= 2:
            self.create_timer(1.0, self._summary_tick)

        # 시작 파라미터 요약
        self.get_logger().info(
            "[init] params: "
            f"input={self.input_topic}, window={self.window_size}, ch={self.num_channels}, fs={self.fs}, "
            f"classes={len(self.class_names)}, probs={self.publish_probs}, debug={self.debug}"
        )

        self.get_logger().info(
            f"[init] pubs: {self.gesture_topic}, {self.gesture_idx_topic}, {self.gesture_conf_topic}"
            + (f", {self.gesture_probs_topic}" if self.publish_probs else "")
        )

    # ---------------- Summary tick (1Hz) ----------------
    def _summary_tick(self):
        now = time.time()
        if self._last_cb_time is None:
            age = "no data yet"
        else:
            age = f"{now - self._last_cb_time:.2f}s ago"
        buf_len = len(self.buffers[0]) if self.buffers else 0
        self.get_logger().info(
            f"[diag] recv={self._recv_count} inf_ok={self._infer_ok} inf_exc={self._infer_exc} pub={self._pub_count} "
            f"buf={buf_len}/{self.window_size} last_cb={age}"
        )
        # 카운터는 누적 유지(재부팅 전까지 누적치가 더 유용)

    # ---------------- Callback ----------------
    def _on_emg(self, msg: Float32MultiArray) -> None:
        self._recv_count += 1
        self._last_cb_time = time.time()

        data = np.asarray(msg.data, dtype=np.float32)

        if data.size != self.num_channels:
            if self.debug >= 1:
                self.get_logger().warn(f"[cb] invalid length={data.size}, expected={self.num_channels}")
            return

        # NaN/Inf 방지
        if not np.isfinite(data).all():
            if self.debug >= 1:
                self.get_logger().warn(f"[cb] non-finite detected (NaN/Inf) -> dropped")
            return

        # 필터 적용
        try:
            filtered = self.filter.apply(data)
        except Exception as e:
            self._infer_exc += 1
            self.get_logger().error(f"[filter] exception: {e}")
            return

        # 버퍼에 push
        try:
            for ch in range(self.num_channels):
                self.buffers[ch].append(float(filtered[ch]))
        except Exception as e:
            self._infer_exc += 1
            self.get_logger().error(f"[buffer] exception: {e}")
            return

        # 윈도우 충족 체크
        if len(self.buffers[0]) < self.window_size:
            if self.debug >= 1 and (len(self.buffers[0]) % 50 == 0):
                self.get_logger().info(f"[cb] filling window... {len(self.buffers[0])}/{self.window_size}")
            return

        # 추론
        try:
            window = np.stack(
                [np.fromiter(self.buffers[ch], dtype=np.float32) for ch in range(self.num_channels)],
                axis=0,
            )  # shape: (8, W)
            tensor = torch.from_numpy(window).unsqueeze(0).to(self._device)  # (1, 8, W)

            with torch.no_grad():
                logits = self.model(tensor)                 # (1, C)
                probs = F.softmax(logits, dim=1)            # (1, C)
                pred_idx = int(torch.argmax(probs, dim=1).item())
                conf = float(probs[0, pred_idx].item())
                label = (
                    self.class_names[pred_idx] if 0 <= pred_idx < len(self.class_names) else str(pred_idx)
                )
            self._infer_ok += 1
        except Exception as e:
            self._infer_exc += 1
            self.get_logger().error(f"[infer] exception: {e}")
            return

        # 퍼블리시
        try:
            out_str = String();   out_str.data = f"{label} ({conf:.2f})"
            self.pub_gesture.publish(out_str)

            out_idx = Int32();    out_idx.data = pred_idx
            self.pub_gesture_idx.publish(out_idx)

            out_conf = Float32(); out_conf.data = conf
            self.pub_gesture_conf.publish(out_conf)

            if self.pub_gesture_probs is not None:
                arr = Float32MultiArray()
                arr.data = probs.squeeze(0).cpu().numpy().astype(np.float32).tolist()
                self.pub_gesture_probs.publish(arr)

            self._pub_count += 1
            if self.debug >= 1:
                self.get_logger().info(f"[pub] {label} ({conf*100:.1f}%) idx={pred_idx}")
        except Exception as e:
            self.get_logger().error(f"[publish] exception: {e}")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmgClassifierNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

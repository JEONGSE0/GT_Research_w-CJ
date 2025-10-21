#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class ShadowHandRelay(Node):
    """
    POSE → POSE 릴레이 노드.
    - input_topic:  PoseStamped 구독 (예: /right_controller)
    - output_topic: PoseStamped 퍼블리시 (예: /controller_translate 또는 /controller_orient)
    - enable_topic: std_msgs/Bool 구독으로 on/off 토글
    - enabled, pos_scale, frame_id_override 파라미터 지원
    - position_only / orientation_only 플래그로 이동/회전 전용 구분
    """
    def __init__(self):
        super().__init__('shadow_hand_relay')

        # -------- Parameters --------
        self.declare_parameter('input_topic', '/right_controller')
        self.declare_parameter('output_topic', '/controller')
        self.declare_parameter('enable_topic', '/shadow_hand_enable')
        self.declare_parameter('enabled', True)
        self.declare_parameter('pos_scale', 1.0)
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('position_only', False)      # 이동 전용 릴레이일 때 True
        self.declare_parameter('orientation_only', False)   # 회전 전용 릴레이일 때 True

        self.input_topic       = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic      = self.get_parameter('output_topic').get_parameter_value().string_value
        self.enable_topic      = self.get_parameter('enable_topic').get_parameter_value().string_value
        self.enabled           = bool(self.get_parameter('enabled').value)
        self.pos_scale         = float(self.get_parameter('pos_scale').value)
        self.frame_id_override = self.get_parameter('frame_id_override').get_parameter_value().string_value
        self.position_only     = bool(self.get_parameter('position_only').value)
        self.orientation_only  = bool(self.get_parameter('orientation_only').value)

        # 상호배타 검증
        if self.position_only and self.orientation_only:
            self.get_logger().warn("Both position_only and orientation_only set. Forcing orientation_only=False.")
            self.orientation_only = False

        # 파라미터 런타임 갱신
        self.add_on_set_parameters_callback(self._on_params)

        # -------- QoS --------
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE)

        # -------- IO --------
        self.pub_pose = self.create_publisher(PoseStamped, self.output_topic, qos)
        self.sub_pose = self.create_subscription(PoseStamped, self.input_topic, self._cb_pose, qos)
        self.sub_en   = self.create_subscription(Bool, self.enable_topic, self._cb_enable, qos)

        self.get_logger().info(
            f"[Relay] {self.input_topic} -> {self.output_topic} "
            f"(enabled={self.enabled}, pos_scale={self.pos_scale}, "
            f"position_only={self.position_only}, orientation_only={self.orientation_only}, "
            f"enable_topic={self.enable_topic})"
        )

    # --- callbacks ---
    def _on_params(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
            elif p.name == 'pos_scale':
                self.pos_scale = float(p.value)
            elif p.name == 'frame_id_override':
                self.frame_id_override = str(p.value)
            elif p.name == 'position_only':
                self.position_only = bool(p.value)
            elif p.name == 'orientation_only':
                self.orientation_only = bool(p.value)
        return SetParametersResult(successful=True)

    def _cb_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        self.get_logger().info(f"[Relay] enabled -> {self.enabled}")

    def _cb_pose(self, msg: PoseStamped):
        if not self.enabled:
            return

        out = PoseStamped()
        # 헤더 복사 + frame_id override
        out.header = msg.header
        if self.frame_id_override:
            out.header.frame_id = self.frame_id_override

        # --- 포지션 처리 ---
        if not self.orientation_only:
            out.pose.position.x = msg.pose.position.x * self.pos_scale
            out.pose.position.y = msg.pose.position.y * self.pos_scale
            out.pose.position.z = msg.pose.position.z * self.pos_scale
        else:
            # 회전 전용일 땐 위치는 0 (OmniGraph에서 translate 입력 안 쓰면 무시되지만 안전하게 0)
            out.pose.position.x = 0.0
            out.pose.position.y = 0.0
            out.pose.position.z = 0.0

        # --- 오리엔테이션 처리 ---
        if not self.position_only:
            out.pose.orientation = msg.pose.orientation
        else:
            # 이동 전용일 땐 단위 quaternion 유지
            out.pose.orientation.w = 1.0
            out.pose.orientation.x = 0.0
            out.pose.orientation.y = 0.0
            out.pose.orientation.z = 0.0

        self.pub_pose.publish(out)

def main():
    rclpy.init()
    node = ShadowHandRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

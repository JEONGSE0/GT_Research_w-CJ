#!/usr/bin/env python3
import sys, os, tty, termios, fcntl, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

HELP = """
[shadow_hand_keyctl]
  t : toggle translate enable
  r : toggle orient    enable
  q : quit
"""

class SafeTTY:
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

class KeyCtl(Node):
    """
    터미널 키로 enable 토픽(Bool)을 토글하는 노드.
    - translate_enable_topic (default: /shadow_hand_enable_translate)
    - orient_enable_topic    (default: /shadow_hand_enable_orient)
    """
    def __init__(self):
        super().__init__('shadow_hand_keyctl')
        self.declare_parameter('translate_enable_topic', '/shadow_hand_enable_translate')
        self.declare_parameter('orient_enable_topic', '/shadow_hand_enable_orient')

        self.t_topic = self.get_parameter('translate_enable_topic').get_parameter_value().string_value
        self.r_topic = self.get_parameter('orient_enable_topic').get_parameter_value().string_value

        self.pub_t = self.create_publisher(Bool, self.t_topic, 1)
        self.pub_r = self.create_publisher(Bool, self.r_topic, 1)

        self.en_t = True
        self.en_r = True
        self._publish()

        if not os.isatty(sys.stdin.fileno()):
            self.get_logger().warn("stdin is not a TTY. Keyboard control won't work when launched without a terminal.")
        else:
            self.get_logger().info(HELP)

        self.timer = self.create_timer(0.02, self._tick)

    def _publish(self):
        self.pub_t.publish(Bool(data=self.en_t))
        self.pub_r.publish(Bool(data=self.en_r))
        self.get_logger().info(f"[keyctl] translate={self.en_t}, orient={self.en_r}")

    def _tick(self):
        if not os.isatty(sys.stdin.fileno()): return
        r,_,_ = select.select([sys.stdin], [], [], 0.0)
        if not r: return
        try:
            ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
        except BlockingIOError:
            return
        if not ch: return

        if ch.lower() == 't':
            self.en_t = not self.en_t
            self._publish()
        elif ch.lower() == 'r':
            self.en_r = not self.en_r
            self._publish()
        elif ch.lower() == 'q':
            self.get_logger().info("bye")
            rclpy.shutdown()

def main():
    rclpy.init()
    with SafeTTY():
        node = KeyCtl()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            # rclpy.shutdown() 는 KeyCtl에서 q 눌렀을 때 호출

if __name__ == '__main__':
    main()

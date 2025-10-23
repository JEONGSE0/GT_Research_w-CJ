#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import threading
from typing import Optional, List
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float32

from bleak import BleakScanner, BleakClient

DEFAULT_SERVICE_UUID = "12345678-1234-5678-1234-123456789abc"
DEFAULT_CHAR_UUID    = "abcdef12-3456-789a-bcde-f123456789ab"
NUM_CHANNELS = 8

def parse_emg_bundle(data: bytes, vref: float = 4.5, gain: int = 24) -> List[List[float]]:
    lsb = 2 * vref / (gain * (2**24 - 1))  # Volt/LSB
    out: List[List[float]] = []
    for i in range(0, len(data), 24):
        chunk = data[i:i+24]
        if len(chunk) < 24:
            break
        ch_vals = []
        for j in range(NUM_CHANNELS):
            raw = chunk[j*3:j*3+3]
            val = int.from_bytes(raw, byteorder='big', signed=False)
            if val & 0x800000:
                val -= 1 << 24  # sign extend
            voltage_uV = float(val) * lsb * 1e6
            ch_vals.append(voltage_uV)
        out.append(ch_vals)
    return out

class EmgBleNode(Node):
    def __init__(self):
        super().__init__('emg_ble_node')

        # --- parameters ---
        self.declare_parameter('address', '')
        self.declare_parameter('name_filter', '')
        self.declare_parameter('service_uuid', DEFAULT_SERVICE_UUID)
        self.declare_parameter('emg_char_uuid', DEFAULT_CHAR_UUID)
        self.declare_parameter('topic', '/emg/signal')
        self.declare_parameter('publish_queue', 50)
        self.declare_parameter('log_found_devices', True)
        self.declare_parameter('drain_period_sec', 0.005)
        self.declare_parameter('publish_per_channel', False)
        self.declare_parameter('per_channel_prefix', '')
        self.declare_parameter('per_channel_decimation', 1)

        # --- get params ---
        self.address       = self.get_parameter('address').get_parameter_value().string_value
        self.name_filter   = self.get_parameter('name_filter').get_parameter_value().string_value
        self.service_uuid  = self.get_parameter('service_uuid').get_parameter_value().string_value
        self.char_uuid     = self.get_parameter('emg_char_uuid').get_parameter_value().string_value
        self.topic         = self.get_parameter('topic').get_parameter_value().string_value
        qdepth             = int(self.get_parameter('publish_queue').get_parameter_value().integer_value)
        self._log_found    = self.get_parameter('log_found_devices').get_parameter_value().bool_value

        drain_val = self.get_parameter('drain_period_sec').value
        try:
            drain_period = float(drain_val)
        except Exception:
            drain_period = 0.005
            self.get_logger().warn(f"Invalid drain_period_sec={drain_val!r}, fallback=0.005s")

        self.publish_per_channel = self.get_parameter('publish_per_channel').get_parameter_value().bool_value
        prefix_param = self.get_parameter('per_channel_prefix').get_parameter_value().string_value
        self.per_ch_prefix = prefix_param if prefix_param else (self.topic + '/data')
        self.per_ch_decim  = max(1, int(self.get_parameter('per_channel_decimation').get_parameter_value().integer_value))

        # --- QoS ---
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=qdepth,
                         reliability=QoSReliabilityPolicy.RELIABLE)

        self.pub_emg = self.create_publisher(Float32MultiArray, self.topic, qos)

        self.pub_ch = []
        if self.publish_per_channel:
            for i in range(NUM_CHANNELS):
                ch_topic = f"{self.per_ch_prefix}{i+1}"  # .../data1..data8
                self.pub_ch.append(self.create_publisher(Float32, ch_topic, qos))
            self.get_logger().info(
                "Per-channel topics enabled: " +
                ", ".join([f"{self.per_ch_prefix}{i+1}" for i in range(NUM_CHANNELS)])
            )
        else:
            self.get_logger().info("Per-channel topics disabled.")

        # --- buffer & timer ---
        self._queue = deque()
        self._queue_lock = threading.Lock()
        self._sample_count = 0
        self.create_timer(drain_period, self._drain_queue)

        # --- asyncio loop (ble) ---
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.loop_thread.start()

        self.client: Optional[BleakClient] = None
        self._stop = False

        asyncio.run_coroutine_threadsafe(self._connect_and_listen(), self.loop)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _find_device_address(self) -> Optional[str]:
        self.get_logger().info('Scanning BLE devices...')
        devices = await BleakScanner.discover(timeout=5.0)
        if self._log_found:
            for d in devices:
                self.get_logger().info(f"Found: {d.name} ({d.address})")
        if self.name_filter:
            key = self.name_filter.lower()
            for d in devices:
                name = (d.name or "").lower()
                if name and key in name:
                    self.get_logger().info(f"Matched: {d.name} ({d.address})")
                    return d.address
        return None

    async def _connect_and_listen(self):
        while not self._stop:
            try:
                addr = self.address or await self._find_device_address()
                if not addr:
                    self.get_logger().warn('No device matched. Retrying in 2s...')
                    await asyncio.sleep(2.0); continue

                self.client = BleakClient(addr)
                self.get_logger().info(f'Connecting to {addr}...')
                await self.client.connect()
                if not getattr(self.client, "is_connected", False):
                    self.get_logger().error('BLE connect failed. Retry...')
                    await asyncio.sleep(2.0); continue

                # optional service/char validation
                try:
                    svcs = None
                    try:
                        svcs = await self.client.get_services()
                    except Exception:
                        svcs = getattr(self.client, "services", None)
                    if self.service_uuid and svcs:
                        svc = next((s for s in svcs if s.uuid.lower()==self.service_uuid.lower()), None)
                        if not svc:
                            self.get_logger().warn(f'Service {self.service_uuid} not found.')
                        else:
                            ch = next((c for c in svc.characteristics if c.uuid.lower()==self.char_uuid.lower()), None)
                            if not ch or 'notify' not in (ch.properties or []):
                                self.get_logger().warn(f'Characteristic {self.char_uuid} not found or no notify.')
                except Exception as e:
                    self.get_logger().warn(f"service check warn: {e}")

                await self.client.start_notify(self.char_uuid, self._on_notify)
                self.get_logger().info('EMG notify started.')

                while getattr(self.client, "is_connected", False):
                    await asyncio.sleep(1.0)

                self.get_logger().warn('BLE disconnected.')
            except Exception as e:
                self.get_logger().error(f'BLE error: {e}')
            finally:
                try:
                    if self.client:
                        try:
                            await self.client.stop_notify(self.char_uuid)
                        except Exception:
                            pass
                        await self.client.disconnect()
                except Exception:
                    pass
                await asyncio.sleep(1.0)

    def _on_notify(self, _handle: int, data: bytes):
        samples = parse_emg_bundle(data)
        if samples:
            with self._queue_lock:
                self._queue.extend(samples)

    def _drain_queue(self):
        to_publish = []
        with self._queue_lock:
            while self._queue:
                to_publish.append(self._queue.popleft())
        for sample in to_publish:
            arr = Float32MultiArray()
            arr.data = [float(x) for x in sample]
            self.pub_emg.publish(arr)

            if self.publish_per_channel:
                self._sample_count += 1
                if (self._sample_count % self.per_ch_decim) == 0:
                    for i, v in enumerate(sample):
                        msg = Float32()
                        msg.data = float(v)
                        self.pub_ch[i].publish(msg)

    def destroy_node(self):
        self._stop = True
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()

def main():
    rclpy.init()
    node = EmgBleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

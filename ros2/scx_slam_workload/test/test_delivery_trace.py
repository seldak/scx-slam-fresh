#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Exercise the actual adapter with delivery tracing disabled and enabled."""
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, Image
from std_srvs.srv import Trigger


def run(adapter, enabled):
    topic = '/trace_test_' + uuid.uuid4().hex
    node = rclpy.create_node('delivery_trace_test_' + uuid.uuid4().hex)
    publisher = node.create_publisher(
        Imu, topic, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
    args = [adapter, '--ros-args', '-p', f'imu_input:={topic}',
            '-p', f'camera_input:={topic}/camera', '-p', f'imu_output:={topic}/jobs',
            '-p', f'camera_output:={topic}/camera_jobs',
            '-p', f'readiness_service:={topic}/ready']
    if enabled:
        args += ['-p', 'trace_delivery:=true']
    process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() != 1:
            if time.monotonic() > deadline or process.poll() is not None:
                raise AssertionError('adapter subscription did not connect')
            rclpy.spin_once(node, timeout_sec=0.05)
        client = node.create_client(Trigger, topic + '/ready')
        assert client.wait_for_service(timeout_sec=5), 'readiness service did not connect'

        def ready():
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result() is not None
            return future.result()

        assert not ready().success, 'must wait for the camera publisher'
        camera = node.create_publisher(
            Image, topic + '/camera',
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        deadline = time.monotonic() + 5
        while not ready().success:
            assert time.monotonic() < deadline, 'matched publishers did not make adapter ready'
        # Readiness queries above must not consume or fabricate sensor messages.
        for ordinal in (1, 2):
            message = Imu()
            message.header.stamp.sec = ordinal
            publisher.publish(message)
        # Let the adapter take both messages before its clean shutdown flush.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5)
        assert process.returncode == 0, output
        assert 'adapter_imu: received=2 published=2 dropped=0' in output, output
        assert 'adapter_camera: received=0 published=0 dropped=0' in output, output
        records = [dict(field.split('=', 1) for field in line.split()[1:])
                   for line in output.splitlines() if line.startswith('delivery_trace:')]
        if not enabled:
            assert not records and 'delivery_trace_summary:' not in output, output
            return
        assert len(records) == 2, output
        assert 'delivery_trace_summary: records=2 omitted=0' in output, output
        for ordinal, record in enumerate(records, 1):
            assert record['stream'] == 'IMU' and int(record['job_id']) == ordinal
            assert int(record['source_ts_ns']) == ordinal * 10**9
            assert record['published'] == '1'
            assert 0 < int(record['entry_mono_ns']) <= int(record['release_mono_ns'])
            assert int(record['release_mono_ns']) <= int(record['exit_mono_ns'])
            assert int(record['entry_real_ns']) > 0
            assert 'rmw_source_ns' in record and 'rmw_received_ns' in record
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        run(sys.argv[1], False)
        run(sys.argv[1], True)
    finally:
        rclpy.shutdown()

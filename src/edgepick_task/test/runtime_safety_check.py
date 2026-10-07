#!/usr/bin/env python3
"""Run built nodes against synthetic ROS messages; never enable physical I2C."""
import contextlib
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]


@contextlib.contextmanager
def process(executable, parameters):
    command = [str(ROOT / 'install/edgepick_task/lib/edgepick_task' / executable), '--ros-args']
    for key, value in parameters.items():
        command += ['-p', f'{key}:={value}']
    with tempfile.TemporaryFile(mode='w+') as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        def output():
            log.seek(0)
            return log.read()
        try:
            yield child, output
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()


def wait(node, predicate, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def adapter_check(node):
    from rclpy.action import ActionServer
    from moveit_msgs.action import MoveGroup
    prefix = '/risk_' + uuid.uuid4().hex
    state = node.create_publisher(String, prefix + '/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    events = []
    subscription = node.create_subscription(String, prefix + '/event',
        lambda msg: events.append(msg.data), 10)
    def unexpected_goal(handle):
        raise AssertionError('No action goal is implemented in this adapter')
    server = ActionServer(node, MoveGroup, prefix + '/move', unexpected_goal)
    try:
        with process('moveit_action_adapter_node', dict(use_mock_action_results='false',
                action_result_delay_ms=500, move_group_action_name=prefix + '/move',
                state_topic=prefix + '/state', event_topic=prefix + '/event')) as (_, output):
            assert wait(node, lambda: state.get_subscription_count()), output()
            state.publish(String(data='planning'))
            assert wait(node, lambda: events), output()
            assert events == ['plan_failed'], events
            assert 'no goal construction' in output(), output()
    finally:
        server.destroy()
        node.destroy_subscription(subscription)
        node.destroy_publisher(state)


def negative_retry_check(node):
    prefix = '/risk_' + uuid.uuid4().hex
    event = node.create_publisher(String, prefix + '/event', 10)
    states = []
    subscription = node.create_subscription(String, prefix + '/state',
        lambda msg: states.append(msg.data), 10)
    try:
        with process('task_node', dict(max_recovery_attempts=-1,
                event_topic=prefix + '/event', state_topic=prefix + '/state')) as (_, output):
            assert wait(node, lambda: event.get_subscription_count()), output()
            event.publish(String(data='start_requested'))
            assert wait(node, lambda: 'perceiving' in states), output()
            event.publish(String(data='timeout'))
            assert wait(node, lambda: 'failed' in states), output()
            assert 'recovering' not in states, states
    finally:
        node.destroy_subscription(subscription)
        node.destroy_publisher(event)


def main():
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/edgepick_safety_ros_logs')
    rclpy.init()
    node = rclpy.create_node('edgepick_safety_check')
    try:
        adapter_check(node)
        print('PASS: real adapter refuses fake success', flush=True)
        negative_retry_check(node)
        print('PASS: negative retry budget', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

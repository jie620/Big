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
from geometry_msgs.msg import PointStamped
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


def executor_check(node, mode):
    prefix = '/risk_' + uuid.uuid4().hex
    state = node.create_publisher(String, prefix + '/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    target = node.create_publisher(PointStamped, prefix + '/target', 10)
    events = []
    subscription = node.create_subscription(String, prefix + '/event',
        lambda msg: events.append(msg.data), 10)
    parameters = dict(use_real_i2c='false', use_kinematics_service='false',
        require_startup_pose_ready='false', tracking_enabled=str(mode == 'duplicate').lower(),
        target_wait_sec='4.0', state_transition_wait_sec='1.5',
        tracking_motion_time_ms=20, pregrasp_motion_time_ms=1000,
        target_point_topic=prefix + '/target', state_topic=prefix + '/state',
        event_topic=prefix + '/event')
    with process('orange_grasp_executor_node', parameters) as (child, output):
        assert wait(node, lambda: state.get_subscription_count() and target.get_subscription_count()), output()
        point = PointStamped()
        point.header.frame_id = 'camera_color_optical_frame'
        point.header.stamp = node.get_clock().now().to_msg()
        point.point.z = 0.5
        if mode == 'stale':
            point.header.stamp.sec -= 5
        if mode == 'wrong_frame':
            point.header.frame_id = 'base_link'
        state.publish(String(data='planning'))
        target.publish(point)
        if mode in ('cancel', 'success'):
            assert wait(node, lambda: 'plan_succeeded' in events), output()
            state.publish(String(data='executing'))
            assert wait(node, lambda: "Sending direct orange grasp step 'pregrasp/open'" in output()), output()
            if mode == 'cancel':
                state.publish(String(data='canceled'))
                assert wait(node, lambda: child.poll() is not None, 1), output()
                assert child.returncode != 0
                assert "Sending direct orange grasp step 'descend'" not in output(), output()
                assert 'execution_succeeded' not in events, events
            else:
                assert wait(node, lambda: 'execution_succeeded' in events, 8), output()
                state.publish(String(data='verifying'))
                assert wait(node, lambda: 'verification_succeeded' in events), output()
                state.publish(String(data='succeeded'))
                assert wait(node, lambda: child.poll() is not None, 6), output()
                assert child.returncode == 0, output()
        else:
            assert wait(node, lambda: child.poll() is not None, 6), output()
            assert child.returncode != 0, output()
            assert 'plan_succeeded' not in events, output()
            assert 'Sending direct orange grasp step' not in output(), output()
    node.destroy_subscription(subscription)
    node.destroy_publisher(state)
    node.destroy_publisher(target)


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


def startup_check():
    with process('startup_pose_sequence_node', dict(use_real_i2c='false',
            zero_motion_time_ms=3000)) as (child, output):
        end = time.monotonic() + 5
        while time.monotonic() < end and 'Sending startup zero/home' not in output():
            time.sleep(0.02)
        assert 'Sending startup zero/home' in output(), output()
        child.send_signal(signal.SIGINT)
        assert child.wait(timeout=3) != 0, output()
        assert 'Sending startup fixed restore' not in output(), output()


def main():
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/edgepick_safety_ros_logs')
    rclpy.init()
    node = rclpy.create_node('edgepick_safety_check')
    try:
        for mode in ('cancel', 'stale', 'wrong_frame', 'duplicate', 'success'):
            executor_check(node, mode)
            print(f'PASS: {mode}', flush=True)
        adapter_check(node)
        print('PASS: real adapter refuses fake success', flush=True)
        negative_retry_check(node)
        print('PASS: negative retry budget', flush=True)
        startup_check()
        print('PASS: startup interruption', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Offline Task 1 regression tests: no imports of hardware modules or controller init.

Only class definitions and literal timeout constants are compiled from the real
controller. All camera, HTTP and STM calls use mocks; no real Manager is started.
"""
import ast
import json
from pathlib import Path
import queue
import subprocess
import threading
import types
from typing import Optional
import unittest
from unittest.mock import Mock, mock_open, patch


class CountingLock:
    def __init__(self):
        self.lock = threading.Lock()
        self.releases = 0

    def acquire(self, **kwargs):
        return self.lock.acquire(**kwargs)

    def release(self):
        self.lock.release()
        self.releases += 1

    def locked(self):
        return self.lock.locked()


class Task1CVLockTests(unittest.TestCase):
    def setUp(self):
        local = Path(__file__).with_name('final_week_8_backup.py')
        source = local if local.exists() else Path(__file__).resolve().parents[1] / 'final_week_8_backup.py'
        tree = ast.parse(source.read_text())
        safe_nodes = [node for node in tree.body if isinstance(node, ast.ClassDef) or (
            isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id in (
                    'ACK_TIMEOUT', 'CAPTURE_ATTEMPTS', 'CAPTURE_TIMEOUT', 'IMAGE_API_TIMEOUT'
                ) for target in node.targets))]
        self.http = Mock()
        self.http.RequestException = RuntimeError
        self.capture = Mock()
        self.env = {
            'Optional': Optional, 'queue': queue, 'json': json,
            'time': types.SimpleNamespace(time=lambda: 123456),
            'subprocess': types.SimpleNamespace(run=self.capture, CalledProcessError=subprocess.CalledProcessError),
            'requests': self.http, 'API_IP_IMAGE': 'mock-cv', 'API_PORT_IMAGE': 8001,
            'SYMBOL_MAP': {},
            'AndroidMessage': lambda cat, value: types.SimpleNamespace(cat=cat, value=value),
        }
        exec(compile(ast.Module(body=safe_nodes, type_ignores=[]), str(source), 'exec'), self.env)
        self.bot = self.env['RaspberryPi'].__new__(self.env['RaspberryPi'])
        self.bot.logger = Mock()
        self.bot.movement_lock = CountingLock()
        self.bot.stm_ack_guard = threading.Lock()
        self.bot.stm_pending_id = types.SimpleNamespace(value=0)
        self.bot.motion_fault = threading.Event()
        self.bot.unpause = threading.Event()
        self.bot.unpause.set()
        self.bot.command_queue = Mock()
        self.bot.android_queue = queue.Queue()
        self.bot.rpi_action_queue = queue.Queue()
        self.bot.path_queue = queue.Queue()
        self.bot.stm_link = Mock()
        self.bot.failed_obstacles = []
        self.bot.success_obstacles = []
        self.bot.obstacles = {1: {'id': 1, 'x': 10, 'y': 10, 'd': 4}}
        self.bot.current_location = {'x': 5, 'y': 5, 'd': 0}
        self.bot.failed_attempt = True
        self.files = patch('builtins.open', mock_open(read_data=b'mocked jpeg'))
        self.files.start()
        self.addCleanup(self.files.stop)

    def response(self, image_id='NA', numeric='NA'):
        response = Mock()
        response.status_code = 200
        response.json.return_value = {'image_id': image_id, 'obstacle_id': numeric}
        return response

    def test_onnx_no_detection_404_is_retried_before_success(self):
        missing = self.response()
        missing.status_code = 404
        self.http.post.side_effect = [missing, self.response('RIGHT', '38')]
        self.run_commands('SNAP1', 'FIN')
        self.assertEqual(self.capture.call_count, 2)
        missing.raise_for_status.assert_not_called()
        missing.json.assert_not_called()
        self.assertEqual(self.bot.success_obstacles, ['1'])
        self.assertEqual(self.bot.movement_lock.releases, 2)

    def run_commands(self, *commands):
        self.bot.command_queue.get.side_effect = [*commands, StopIteration]
        with self.assertRaises(StopIteration):
            self.bot.command_follower()

    def test_seven_na_retries_finish_before_fin_and_release_once(self):
        def predict(*args, **kwargs):
            self.assertTrue(self.bot.movement_lock.locked())
            # FIN has not even been dequeued while camera retries are active.
            self.assertEqual(self.bot.command_queue.get.call_count, 1)
            return self.response()
        self.http.post.side_effect = predict
        self.run_commands('SNAP1_C', 'FIN')
        self.assertEqual(self.http.post.call_count, 7)
        self.assertEqual(self.capture.call_count, 7)
        self.assertEqual(self.bot.failed_obstacles, ['1'])
        self.assertEqual(self.bot.movement_lock.releases, 2)  # SNAP then FIN
        self.assertFalse(self.bot.movement_lock.locked())
        self.assertFalse(self.bot.motion_fault.is_set())
        statuses = [message.value for message in self.bot.android_queue.queue if message.cat == 'status']
        self.assertEqual(statuses, ['finished'])

    def test_valid_recognition_updates_tablet_before_unlock(self):
        self.http.post.side_effect = [self.response(), self.response('LEFT', '39')]
        self.bot.failed_obstacles = ['1']
        self.bot.movement_lock.acquire()
        self.bot.snap_and_rec('1')
        self.assertTrue(self.bot.movement_lock.locked())
        self.assertEqual(self.bot.movement_lock.releases, 0)
        self.assertEqual(self.bot.failed_obstacles, [])
        self.assertEqual(self.bot.success_obstacles, ['1'])
        targets = [m.value for m in self.bot.android_queue.queue if m.cat == 'image-rec']
        self.assertEqual(targets, [{'obstacle_id': '1', 'image_id': '39'}])
        self.assertEqual(self.http.post.call_args.kwargs['timeout'], (3, 10))
        self.assertEqual(self.capture.call_args.kwargs['timeout'], 10)

    def test_fin_retry_sees_failed_capture_before_requesting_new_path(self):
        self.bot.failed_attempt = False
        self.http.post.return_value = self.response()
        self.bot.request_algo = Mock()
        self.run_commands('SNAP1', 'FIN')
        self.assertEqual(self.bot.failed_obstacles, ['1'])
        self.bot.request_algo.assert_called_once_with(
            {'obstacles': [self.bot.obstacles[1]], 'mode': '0'}, 5, 5, 0, retrying=True)
        self.assertFalse(any(m.value == 'finished' for m in self.bot.android_queue.queue))
        self.assertEqual(self.bot.movement_lock.releases, 2)

    def test_camera_error_records_failure_and_releases_via_caller(self):
        self.capture.side_effect = subprocess.TimeoutExpired('mock-camera', 10)
        self.run_commands('SNAP1', 'FIN')
        self.assertEqual(self.bot.failed_obstacles, ['1'])
        self.assertEqual(self.bot.movement_lock.releases, 2)
        self.http.post.assert_not_called()

    def test_http_error_does_not_crash_or_double_release(self):
        response = self.response()
        response.raise_for_status.side_effect = RuntimeError('HTTP 500')
        self.http.post.return_value = response
        self.run_commands('SNAP1', 'FIN')
        self.assertEqual(self.bot.failed_obstacles, ['1'])
        self.assertEqual(self.bot.movement_lock.releases, 2)

    def test_network_timeout_does_not_crash_or_double_release(self):
        self.http.post.side_effect = TimeoutError('mock timeout')
        self.run_commands('SNAP1', 'FIN')
        self.assertEqual(self.bot.failed_obstacles, ['1'])
        self.assertEqual(self.bot.movement_lock.releases, 2)

    def test_malformed_api_response_records_failure(self):
        response = self.response()
        response.json.return_value = {'unexpected': True}
        self.http.post.return_value = response
        self.run_commands('SNAP1', 'FIN')
        self.assertEqual(self.bot.failed_obstacles, ['1'])
        self.assertEqual(self.bot.movement_lock.releases, 2)

    def test_missing_ack_never_dispatches_next_command(self):
        self.bot.movement_lock = Mock()
        self.bot.movement_lock.acquire.return_value = False
        self.bot.command_queue.get.return_value = 'center,0,forward,10'
        self.bot.command_follower()
        self.bot.stm_link.send.assert_not_called()
        self.bot.movement_lock.release.assert_not_called()
        self.assertTrue(self.bot.motion_fault.is_set())
        self.assertFalse(self.bot.unpause.is_set())

    def test_missing_ack_never_reports_fin(self):
        self.bot.movement_lock = Mock()
        self.bot.movement_lock.acquire.return_value = False
        self.bot.command_queue.get.return_value = 'FIN'
        self.bot.command_follower()
        self.assertFalse(any(m.value == 'finished' for m in self.bot.android_queue.queue))
        self.bot.movement_lock.release.assert_not_called()

    def test_unexpected_ack_cannot_unlock_camera(self):
        self.bot.movement_lock.acquire()
        self.bot.stm_link.last_done_id = 7
        self.bot.stm_link.recv.side_effect = ['ACK !7/DONE;', StopIteration]
        with self.assertRaises(StopIteration):
            self.bot.recv_stm()
        self.assertTrue(self.bot.movement_lock.locked())
        self.assertEqual(self.bot.movement_lock.releases, 0)

    def test_matching_ack_releases_once_despite_duplicate(self):
        self.bot.movement_lock.acquire()
        self.bot.stm_pending_id.value = 7
        self.bot.stm_link.last_done_id = 7
        self.bot.stm_link.recv.side_effect = ['ACK !7/DONE;', 'ACK !7/DONE;', StopIteration]
        with self.assertRaises(StopIteration):
            self.bot.recv_stm()
        self.assertEqual(self.bot.movement_lock.releases, 1)
        self.assertEqual(self.bot.stm_pending_id.value, 0)

    def test_other_command_ack_cannot_release_pending_move(self):
        self.bot.movement_lock.acquire()
        self.bot.stm_pending_id.value = 8
        self.bot.stm_link.last_done_id = 7
        self.bot.stm_link.recv.side_effect = ['ACK !7/DONE;', StopIteration]
        with self.assertRaises(StopIteration):
            self.bot.recv_stm()
        self.assertEqual(self.bot.stm_pending_id.value, 8)
        self.assertTrue(self.bot.movement_lock.locked())

    def test_motor_send_publishes_pending_id(self):
        self.bot.stm_link.send.return_value = 42
        self.run_commands('center,0,forward,10')
        self.assertEqual(self.bot.stm_pending_id.value, 42)
        self.assertTrue(self.bot.movement_lock.locked())

    def test_second_run_still_handles_camera(self):
        self.http.post.return_value = self.response('LEFT', '39')
        self.run_commands('SNAP1', 'FIN')
        self.bot.unpause.set()
        self.run_commands('SNAP2', 'FIN')
        self.assertEqual(self.bot.success_obstacles, ['1', '2'])
        self.assertEqual(self.bot.movement_lock.releases, 4)

    def test_stitch_network_error_returns_without_killing_worker(self):
        self.http.get.side_effect = RuntimeError('network down')
        self.bot.request_stitch()
        self.assertEqual(self.http.get.call_args.kwargs['timeout'], (3, 10))


if __name__ == '__main__':
    unittest.main()

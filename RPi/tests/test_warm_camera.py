"""Offline tests for the always-on Task 1 camera (no Pi hardware needed)."""
import sys
import types
import unittest
from unittest.mock import Mock, patch

import camera


class FakePicamera2:
    instances = []

    def __init__(self):
        self.options = {}
        self.started = False
        self.closed = False
        self.saved = []
        self.fail_capture = False
        FakePicamera2.instances.append(self)

    def create_still_configuration(self):
        return {'still': True}

    def configure(self, config):
        self.config = config

    def start(self):
        self.started = True

    def capture_file(self, path):
        if self.fail_capture:
            raise RuntimeError('sensor gone')
        self.saved.append(path)

    def stop(self):
        self.started = False

    def close(self):
        self.closed = True


def fake_module():
    return types.SimpleNamespace(Picamera2=FakePicamera2)


@patch.object(camera.time, 'sleep', lambda s: None)
class WarmCameraTests(unittest.TestCase):
    def setUp(self):
        FakePicamera2.instances.clear()

    def test_camera_opens_once_and_every_capture_reuses_it(self):
        with patch.dict(sys.modules, {'picamera2': fake_module()}):
            cam = camera.WarmCamera(Mock())
        self.assertTrue(cam.capture('a.jpg'))
        self.assertTrue(cam.capture('b.jpg'))
        self.assertEqual(len(FakePicamera2.instances), 1)
        fake = FakePicamera2.instances[0]
        self.assertEqual(fake.saved, ['a.jpg', 'b.jpg'])
        self.assertEqual(fake.config, {'still': True})  # full-res still, as in the 3/3 run
        self.assertEqual(fake.options['quality'], 93)

    def test_missing_picamera2_means_fallback(self):
        with patch.dict(sys.modules, {'picamera2': None}):
            cam = camera.WarmCamera(Mock())
        self.assertFalse(cam.capture('a.jpg'))

    def test_configuration_failure_releases_sensor_before_fallback(self):
        with patch.dict(sys.modules, {'picamera2': fake_module()}), \
                patch.object(FakePicamera2, 'configure', side_effect=RuntimeError('configuration failed')):
            cam = camera.WarmCamera(Mock())
        self.assertTrue(FakePicamera2.instances[0].closed)
        self.assertFalse(cam.capture('a.jpg'))

    def test_stop_failure_still_closes_sensor(self):
        with patch.dict(sys.modules, {'picamera2': fake_module()}):
            cam = camera.WarmCamera(Mock())
        fake = FakePicamera2.instances[0]
        with patch.object(fake, 'stop', side_effect=RuntimeError('stop failed')):
            cam.close()
        self.assertTrue(fake.closed)
        self.assertFalse(cam.capture('a.jpg'))

    def test_failed_capture_closes_camera_and_falls_back_for_good(self):
        with patch.dict(sys.modules, {'picamera2': fake_module()}):
            cam = camera.WarmCamera(Mock())
        fake = FakePicamera2.instances[0]
        fake.fail_capture = True
        self.assertFalse(cam.capture('a.jpg'))
        self.assertTrue(fake.closed)
        self.assertFalse(cam.capture('b.jpg'))


if __name__ == '__main__':
    unittest.main()

"""Recorded-map regression in Flask's test client, never the live robot."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ['YOLO_AUTOINSTALL'] = 'false'
from main import app
import consts


class PathApiTests(unittest.TestCase):
    def test_status_identifies_the_loaded_profile_and_speed(self):
        response = app.test_client().get('/status')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {'status': 'ok',
                                        'movement_profile': consts.MOVEMENT_PROFILE,
                                        'recommended_motor_speed': consts.RECOMMENDED_MOTOR_SPEED,
                                        'reverse_calibration': consts.REVERSE_CALIBRATION,
                                        'forward_calibration': consts.FORWARD_CALIBRATION})

    @unittest.skipUnless(consts.MOVEMENT_PROFILE == 'original' and consts.REVERSE_CALIBRATION == 'original' and consts.FORWARD_CALIBRATION == 'original', 'original-only golden maps')
    def test_recorded_maps_keep_original_turns_and_ten_cm_scaling(self):
        golden = Path(__file__).resolve().parents[2] / 'RPi-services/golden/expected_path.json'
        for case in json.loads(golden.read_text()):
            with self.subTest(payload=case['payload']), contextlib.redirect_stdout(io.StringIO()):
                response = app.test_client().post('/path', data=case['payload'], content_type='text/plain')
                self.assertEqual(response.status_code, 200)
                self.assertIsNone(response.json['error'])
                self.assertEqual(response.json['data']['stm_commands'], case['stm_commands'])

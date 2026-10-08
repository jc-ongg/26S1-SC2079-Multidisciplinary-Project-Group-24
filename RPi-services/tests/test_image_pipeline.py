"""Offline CV -> persisted evidence -> Pi numeric-ID contract checks."""
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from PIL import Image
from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'RPi'))
from recognition import normalize_recognition
from communication.android_protocol import to_frame

spec = importlib.util.spec_from_file_location('onnx_api', ROOT / 'RPi-services/image-api/main.py')
api = importlib.util.module_from_spec(spec)
spec.loader.exec_module(api)


def box(cls, conf, xyxy):
    return SimpleNamespace(cls=[cls], conf=[conf], xyxy=np.array([xyxy]))


class ImagePipelineTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.model = SimpleNamespace(boxes=[box(21, .93, [50, 40, 180, 200])])
        self.model.predict = lambda *args, **kwargs: [SimpleNamespace(boxes=self.model.boxes)]
        self.client = TestClient(api.create_app(self.model, self.folder.name))
        self.addCleanup(self.client.close)
        frame = io.BytesIO()
        Image.new('RGB', (320, 240), 'white').save(frame, 'JPEG')
        self.frame = frame.getvalue()

    def capture(self, run, obstacle, signal='C'):
        return self.client.post('/predict', data={
            'run_id': run, 'target_obstacle_id': obstacle, 'signal': signal, 'attempt': '1'},
            files={'image': ('frame.jpg', self.frame, 'image/jpeg')})

    def test_scan_contract_raw_na_and_run_stitch_survive_restart(self):
        first = self.capture('run-a', '1')
        self.assertEqual(first.status_code, 200)
        result = normalize_recognition(first.json(), '1')
        self.assertEqual(to_frame('image-rec', {'obstacle_id': '1', 'image_id': result['obstacle_id']}), '//TARGET,1,38//')
        self.assertEqual(self.client.get(first.json()['raw_url']).status_code, 200)
        self.capture('run-a', '2')  # Same symbol on another obstacle still gets a tile.
        self.model.boxes = []
        missed = self.capture('run-a', '3')
        self.assertFalse(missed.json()['accepted'])
        self.assertTrue((Path(self.folder.name) / missed.json()['raw_file']).is_file())
        stitched = self.client.get('/stitch', params={'run_id': 'run-a'})
        self.assertEqual(stitched.status_code, 200)
        self.assertEqual(Image.open(io.BytesIO(stitched.content)).size, (960, 360))
        self.model.boxes = [box(22, .9, [50, 40, 180, 200])]
        self.capture('run-b', '1')
        restarted = TestClient(api.create_app(self.model, self.folder.name))
        self.addCleanup(restarted.close)
        self.assertEqual(len(restarted.get('/results', params={'run_id': 'run-a'}).json()['captures']), 3)
        self.assertEqual(Image.open(io.BytesIO(restarted.get('/stitch').content)).size, (480, 360))
        self.assertEqual(restarted.get('/stitch', params={'run_id': 'missing'}).status_code, 404)

    def test_multiple_detections_below_half_confidence_return_correct_side(self):
        self.model.boxes = [box(20, .40, [10, 40, 110, 180]), box(21, .45, [200, 40, 300, 180])]
        left = self.capture('run-a', '1', 'L')
        right = self.capture('run-a', '2', 'R')
        self.assertEqual(left.status_code, 200)
        self.assertEqual(left.json()['mdp_image_id'], 39)
        self.assertEqual(right.json()['mdp_image_id'], 38)
        # The former React API returned class index 21 in obstacle_id.
        self.assertEqual(normalize_recognition({'results': [{'image_id': 'RIGHT', 'obstacle_id': 21, 'conf': .9}]})['obstacle_id'], '38')
        with self.assertRaises(ValueError):
            normalize_recognition(left.json(), '2')


if __name__ == '__main__':
    unittest.main()

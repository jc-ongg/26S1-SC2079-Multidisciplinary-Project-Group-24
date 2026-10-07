import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cv_trial_server import create_app, validate_names


def box(class_index, confidence=0.9, coordinates=(5, 5, 60, 60)):
    return SimpleNamespace(cls=[class_index], conf=[confidence], xyxy=np.array([coordinates]))


class FakeModel:
    task = "detect"
    names = {0: "20", 1: "NA", 2: "38", 3: "11"}

    def __init__(self, boxes):
        self.boxes = boxes
        self.calls = 0

    def predict(self, *args, **kwargs):
        self.calls += 1
        return [SimpleNamespace(boxes=self.boxes)]


class CVTrialTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.model = FakeModel([box(2)])
        self.client = create_app(self.model, "best.pt", self.folder.name).test_client()

    def post_image(self, client=None, run_id='legacy'):
        buffer = io.BytesIO()
        Image.new("RGB", (100, 100), "white").save(buffer, format="JPEG")
        buffer.seek(0)
        return (client or self.client).post("/predict", data={"image": (buffer, "../capture.jpg"), 'run_id': run_id})

    def test_numeric_model_label_not_class_index_is_sent(self):
        response = self.post_image()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json["image_id"], "RIGHT")
        self.assertEqual(response.json["obstacle_id"], "38")
        self.assertEqual(response.json["xyxy"], [5.0, 5.0, 60.0, 60.0])

    def test_bullseye_is_not_a_target(self):
        self.model.boxes = [box(1)]
        self.assertEqual(self.post_image().json["image_id"], "NA")

    def test_largest_target_wins_and_bullseye_is_ignored(self):
        self.model.boxes = [box(1, coordinates=(0, 0, 99, 99)), box(0),
                            box(3, coordinates=(10, 10, 90, 90))]
        self.assertEqual(self.post_image().json["obstacle_id"], "11")

    def test_low_confidence_is_not_a_target(self):
        self.model.boxes = [box(2, confidence=0.1)]
        self.assertEqual(self.post_image().json["image_id"], "NA")

    def test_invalid_and_missing_uploads_do_not_run_model(self):
        self.assertEqual(self.client.post("/predict").status_code, 400)
        response = self.client.post("/predict", data={"image": (io.BytesIO(b"bad"), "bad.jpg")})
        self.assertEqual(response.status_code, 400)
        self.assertEqual(self.model.calls, 0)

    def test_health_and_stitch(self):
        self.assertTrue(self.client.get("/health").json["temporary"])
        self.assertEqual(self.client.get("/stitch").status_code, 404)
        self.post_image()
        response = self.client.get("/stitch")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.mimetype, "image/jpeg")
        self.assertEqual(len(list(Path(self.folder.name).glob("*.jpg"))), 3)
        saved = Path(self.folder.name) / response.headers['X-Stitch-File']
        self.assertTrue(saved.is_file())
        with Image.open(saved) as stitched:
            self.assertEqual(stitched.size, (480, 360))

    def test_unknown_model_labels_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_names({0: "person"})

    def test_output_directory_removed_after_startup_is_recreated(self):
        Path(self.folder.name).rmdir()  # empty test-owned directory only
        self.assertEqual(self.post_image().status_code, 200)
        self.assertEqual(len(list(Path(self.folder.name).glob("*.jpg"))), 2)

    def test_inference_errors_are_reported(self):
        def fail(*args, **kwargs):
            raise RuntimeError("test inference failure")
        self.model.predict = fail
        response = self.post_image()
        self.assertEqual(response.status_code, 500)
        self.assertIn("error", response.json)

    def test_stitch_does_not_mix_previous_run_or_duplicate_characters(self):
        self.post_image(run_id='first')  # RIGHT
        self.model.boxes = [box(0)]  # A, current run
        self.post_image(run_id='second')
        self.post_image(run_id='second')  # re-capture of same character
        response = self.client.get('/stitch')
        with Image.open(io.BytesIO(response.data)) as stitched:
            self.assertEqual(stitched.size, (480, 360))
        self.model.boxes = [box(3)]  # another character, same current run
        self.post_image(run_id='second')
        response = self.client.get('/stitch?run_id=second')
        with Image.open(io.BytesIO(response.data)) as stitched:
            self.assertEqual(stitched.size, (960, 360))

    def test_empty_new_run_cannot_show_old_results(self):
        self.post_image(run_id='first')
        self.model.boxes = [box(1)]  # only bullseye/NA
        self.post_image(run_id='second')
        self.assertEqual(self.client.get('/stitch').status_code, 404)
        self.assertEqual(self.client.get('/stitch?run_id=first').status_code, 200)


if __name__ == "__main__":
    unittest.main()

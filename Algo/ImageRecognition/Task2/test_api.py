import io
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from PIL import Image
from fastapi.testclient import TestClient
import main

class SelectorTests(unittest.TestCase):
    def test_empty_is_na(self):
        self.assertEqual(main.select_arrow([])['image_id'], 'NA')
    def test_threshold_and_low_candidate(self):
        d={'class':'left','confidence':.499,'xyxy':[1,2,3,4]}
        response=main.select_arrow([d]);self.assertEqual(response['image_id'],'NA')
        self.assertEqual(response['candidate']['confidence'],.499)
        d['confidence']=.5;self.assertEqual(main.select_arrow([d])['image_id'],'LEFT')
    def test_large_low_box_cannot_override_arrow(self):
        response=main.select_arrow([
            {'class':'left','confidence':.28,'xyxy':[0,0,640,480]},
            {'class':'right','confidence':.90,'xyxy':[30,290,230,480]}])
        self.assertEqual(response['image_id'],'RIGHT')
        self.assertEqual(response['obstacle_id'],38)

class ApiReplayTests(unittest.TestCase):
    def test_real_model_bridge_contract_and_bad_uploads(self):
        root=Path(__file__).resolve().parent.parent
        originals=root/'KC_image_api_v11i/diagnostics/2026-10-14-replay'
        files=[originals/'failed_single_O2.jpg', originals/'successful_control_O2.jpg']+sorted((originals/'failed_run_O2').glob('*.jpg'))
        if not main.MODEL.is_file() or len(files) != 8 or not all(p.is_file() for p in files):
            self.skipTest('Real-model replay requires the external checkpoint and eight saved photos')
        self.assertEqual(len(files),8)
        with TemporaryDirectory() as tmp, patch.object(main,'RESULTS',Path(tmp)), TestClient(main.app) as client:
            health=client.get('/health').json()
            self.assertEqual(health['total_classes'],2)
            self.assertEqual(health['sha256'],main.EXPECTED_SHA256)
            for path in files:
                response=client.post('/predict',files={'image':(path.name,path.read_bytes(),'image/jpeg')})
                self.assertEqual(response.status_code,200,response.text)
                data=response.json();self.assertEqual(data['image_id'],'RIGHT')
                self.assertGreaterEqual(data['conf'],.5)
                print(path.name,round(data['conf'],4))
            self.assertEqual(len(list(Path(tmp).glob('*_result.json'))),8)
            self.assertEqual(client.post('/predict',files={'image':('x.txt',b'bad','text/plain')}).status_code,400)
            self.assertEqual(client.post('/predict',files={'image':('x.jpg',b'bad','image/jpeg')}).status_code,400)
            self.assertEqual(client.post('/predict',files={'image':('x.jpg',b'x'*(main.MAX_UPLOAD_BYTES+1),'image/jpeg')}).status_code,413)

if __name__=='__main__':unittest.main()

import importlib.util
import queue
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import KC_task2_stm32_bridge as bridge

class BridgeTests(unittest.TestCase):
    def test_dry_run_and_format(self):
        self.assertEqual(bridge.main([]), 0)
        self.assertEqual(bridge.frame(12,'MOTOR','TASK2KC',50,1), b':12/MOTOR/TASK2KC/50/1;')

    def test_confidence_and_cancellation(self):
        vision=types.SimpleNamespace(backends=[('image-api','http://test')], _query=lambda *a: ('left',.5))
        with tempfile.TemporaryDirectory() as root, patch.object(bridge.subprocess,'run') as capture:
            self.assertEqual(bridge.capture_arrow(vision,Path(root)/'ok',1,threading.Event()),'left')
            self.assertEqual(capture.call_count,1)
            vision._query=lambda *a: ('right',.49)
            self.assertEqual(bridge.capture_arrow(vision,Path(root)/'low',2,threading.Event()), 'right')
            cancelled=threading.Event();cancelled.set()
            with self.assertRaises(RuntimeError): bridge.capture_arrow(vision,Path(root)/'cancel',1,cancelled)

    def test_early_miss_retries_after_arrival(self):
        stationary=threading.Event()
        calls=[]
        def query(*args):
            calls.append(1)
            if len(calls)==2:stationary.set()
            return ('right',.8) if len(calls)==3 else (None,0)
        vision=types.SimpleNamespace(backends=[('image-api','http://test')],_query=query)
        with tempfile.TemporaryDirectory() as root, patch.object(bridge.subprocess,'run'):
            self.assertEqual(bridge.capture_arrow(vision,Path(root)/'early',2,threading.Event(),stationary),'right')
        self.assertEqual(len(calls),3)

    def test_early_success_without_stationary_wait(self):
        vision=types.SimpleNamespace(backends=[('image-api','http://test')],_query=lambda *a:('left',.9))
        with tempfile.TemporaryDirectory() as root, patch.object(bridge.subprocess,'run'):
            self.assertEqual(bridge.capture_arrow(vision,Path(root)/'early',1,threading.Event(),threading.Event()),'left')

    def test_identification_completion_and_stop(self):
        class Port:
            def __init__(self,*a,**kw): self.q=queue.Queue();self.sent=[];self.closed=False;self.start_seen=False
            def write(self,data):
                if b'/T2/PING/' in data: assert self.start_seen, "Heartbeat arrived before START acknowledgement"
                self.sent.append(data)
                if b'/T2/INFO/' in data: self.q.put(b'!KC_T2/BUILD/2/0;')
                if b'/MOTOR/TASK2KC/' in data:
                    token=data.split(b'/')[0][1:];self.q.put(b'!KC_T2/BUILD_TASK2KC/2.00/0.00;');self.q.put(b'!'+token+b'/DONE;')
            def flush(self):pass
            def read(self,*a):
                try:
                    data=self.q.get(timeout=.01)
                    if b'/BUILD_TASK2KC/' in data:self.start_seen=True
                    return data
                except queue.Empty:return b''
            def close(self):self.closed=True
        port=Port()
        serial=types.SimpleNamespace(Serial=lambda *a,**kw:port)
        vision=types.SimpleNamespace(VisionClient=lambda *a,**kw:None)
        with tempfile.TemporaryDirectory() as root, patch.dict(sys.modules,{'serial':serial,'KC_stm32_link':vision}), patch.object(bridge,'Path',side_effect=lambda s:Path(root)/s):
            self.assertEqual(bridge.main(['--execute']),0)
        self.assertTrue(port.closed)
        self.assertIn(b'/MOTOR/STOP/0/0;',port.sent[-1])
        self.assertTrue(any(b'/MOTOR/TASK2KC/50/1;' in x for x in port.sent))

if __name__=='__main__':unittest.main()

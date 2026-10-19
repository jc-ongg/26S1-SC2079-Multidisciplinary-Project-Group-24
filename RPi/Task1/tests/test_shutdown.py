"""Exercise a real interrupted wait/worker shutdown on Linux, with no hardware."""
import os
from pathlib import Path
import subprocess
import sys
import unittest


class ShutdownTests(unittest.TestCase):
    @unittest.skipUnless(os.name == 'posix', 'Linux/Pi multiprocessing interrupt regression')
    def test_ctrl_c_during_reconnect_wait_stops_workers_and_manager(self):
        script = r'''
import logging, os, signal, sys, threading, time, types
from unittest.mock import Mock, patch
sys.modules['serial'] = types.SimpleNamespace(Serial=lambda *a, **k: (_ for _ in ()).throw(AssertionError('No hardware')))
sys.modules['logger'] = types.SimpleNamespace(prepare_logger=lambda: logging.getLogger('offline-shutdown'))
import task1_controller as controller
from multiprocessing import Process
with patch.object(controller, 'AndroidLink'), patch.object(controller, 'STMLink'):
    bot = controller.RaspberryPi()
worker = Process(target=controller.run_worker, args=(bot.unpause.wait,))
bot.proc_command_follower = worker
worker.start()
time.sleep(.1)
os.kill(worker.pid, signal.SIGINT)
time.sleep(.05)
assert worker.is_alive(), 'Worker must defer Ctrl-C handling to parent'
timer = threading.Timer(.2, lambda: os.kill(os.getpid(), signal.SIGINT))
timer.start()
try:
    bot.android_dropped.wait()
    raise AssertionError('Expected interrupt')
except KeyboardInterrupt:
    bot.stop()
finally:
    timer.join()
assert not worker.is_alive()
assert not bot.manager._process.is_alive()
print('Clean interrupt shutdown; no hardware opened')
'''
        result = subprocess.run([sys.executable, '-B', '-c', script],
            cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn('Traceback', result.stderr)
        self.assertIn('Clean interrupt shutdown', result.stdout)

"""Run only curated mock-based tests; never discover root hardware experiments.

Usage from repository root: python RPi/tests/test_offline.py
Serial construction is blocked even if pyserial is installed.
"""
import logging
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


def blocked_serial(*args, **kwargs):
    raise AssertionError("Offline tests must not open a serial port")


def main():
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    serial = types.ModuleType("serial")
    serial.Serial = blocked_serial
    logger = types.ModuleType("logger")
    logger.prepare_logger = lambda: logging.getLogger("offline")
    with patch.dict(sys.modules, {"serial": serial, "logger": logger}):
        suite = unittest.defaultTestLoader.discover(str(root / "tests"), pattern="test_*.py")
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())

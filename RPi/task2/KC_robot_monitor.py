#!/usr/bin/env python3
"""Pi-hosted STM32 monitor. One serial owner for manual controls and O1 stages."""
import argparse
from collections import deque
from dataclasses import asdict
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

from KC_stm32_link import STM32Link, Task2Error
import KC_task2_o1_calibration as o1

COMMANDS = {'FWD', 'REVS', 'TURNL', 'TURNR', 'REVL', 'REVR'}


class StageLink:
    """Uses the existing serial link; cancellation and sending share a lock."""
    def __init__(self, controller, cancel):
        self.controller = controller
        self.cancel = cancel

    def __getattr__(self, name):
        return getattr(self.controller.link, name)

    def send_motor_async(self, command, speed, value):
        with self.controller.lock:
            if self.cancel.is_set() and command != 'STOP':
                raise Task2Error('Stage cancelled by STOP')
            cmd_id = self.controller.link.send_motor_async(command, speed, value)
            self.controller.event(f'TX :{cmd_id}/MOTOR/{command}/{speed}/{value};')
            return cmd_id

    def stop(self, timeout=5):
        try:
            cmd_id = self.send_motor_async('STOP', 0, 0)
            result = self.wait_result(cmd_id, timeout)
            if result.status != 'DONE':
                raise Task2Error(f'STOP failed: {result}')
        except Exception:
            with self.controller.lock:
                self.controller.mode = 'stop-unconfirmed'
            raise


class Controller:
    def __init__(self, link, laptop_ip):
        self.link = link
        self.laptop_ip = laptop_ip
        self.lock = threading.RLock()
        self.events = deque(maxlen=150)
        self.sequence = 0
        self.mode = 'idle'
        self.cancel = None
        self.last_error = None
        self.stage_thread = None
        self.stop_pending = 0
        self.manual_id = None

    def event(self, message):
        with self.lock:
            self.sequence += 1
            self.events.append(dict(id=self.sequence, time=time.strftime('%H:%M:%S'), message=message))

    def status(self):
        _, sensor = self.link.sensor_snapshot()
        age = None if sensor is None else time.monotonic() - sensor.received_at
        data = None if sensor is None else asdict(sensor)
        if data:
            # JSON cannot represent NaN/Infinity. Surface failed values as null.
            data = {key: (None if isinstance(value, float) and not math.isfinite(value) else value)
                    for key, value in data.items()}
        with self.lock:
            return dict(mode=self.mode, connected=age is not None and age < 1,
                        sensor=data, age_s=age, events=list(self.events), error=self.last_error,
                        stop_pending=self.stop_pending)

    def require_fresh(self):
        _, sensor = self.link.sensor_snapshot()
        if sensor is None or time.monotonic() - sensor.received_at > 1:
            raise Task2Error('STM32 telemetry is stale; movement refused')

    def manual(self, data):
        command = data.get('command')
        speed, value = data.get('speed'), data.get('value')
        if command not in COMMANDS:
            raise ValueError('Unsupported command')
        if type(speed) is not int or not 1 <= speed <= 100:
            raise ValueError('Speed must be an integer from 1 to 100')
        limit = 200 if command in ('FWD', 'REVS') else 360
        if type(value) is not int or not 1 <= value <= limit:
            raise ValueError(f'Distance/angle must be an integer from 1 to {limit}')
        with self.lock:
            if self.mode != 'idle' or self.stop_pending:
                raise Task2Error('Movement or stage is active; use STOP first')
            self.require_fresh()
            cmd_id = self.link.send_motor_async(command, speed, value)
            self.mode = 'manual'
            self.manual_id = cmd_id
            self.last_error = None
            self.event(f'TX :{cmd_id}/MOTOR/{command}/{speed}/{value};')
            threading.Thread(target=self.finish_manual, args=(cmd_id,), daemon=True).start()
        return dict(cmd_id=cmd_id)

    def finish_manual(self, cmd_id):
        try:
            deadline = time.monotonic() + 30
            while True:
                result = self.link.result_now(cmd_id)
                if result is not None:
                    break
                self.require_fresh()
                if time.monotonic() > deadline:
                    raise Task2Error('Manual movement deadline exceeded')
                time.sleep(0.05)
            self.event(f'RX {result.cmd_id}/{result.status}' + (f'/{result.reason}' if result.reason else ''))
            if result.status == 'ERROR':
                with self.lock:
                    self.last_error = result.reason
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
            self.event(str(exc))
            try:
                self.stop()
            except Exception as stop_error:
                self.event(f'STOP failed: {stop_error}')
        finally:
            with self.lock:
                if self.mode == 'manual' and self.manual_id == cmd_id:
                    self.mode = 'idle'
                    self.manual_id = None

    def stop(self):
        with self.lock:
            if self.cancel:
                self.cancel.set()
            self.stop_pending += 1
        try:
            with self.lock:
                cmd_id = self.link.send_motor_async('STOP', 0, 0)
                self.event(f'TX :{cmd_id}/MOTOR/STOP/0/0;')
            result = self.link.wait_result(cmd_id, timeout=5)
            self.event(f'RX {cmd_id}/{result.status}')
            if result.status != 'DONE':
                raise Task2Error('STOP not confirmed')
            return dict(cmd_id=cmd_id, status=result.status)
        except Exception:
            with self.lock:
                self.mode = 'stop-unconfirmed'
            raise
        finally:
            with self.lock:
                self.stop_pending -= 1
                if self.mode == 'stop-unconfirmed' and 'result' in locals() and result.status == 'DONE':
                    self.mode = 'idle'

    def start_stage(self, stage):
        if stage not in ('first-turn', 'second-turn', 'bypass', 'rough-bypass', 'rough-centre', 'o2-prepare'):
            raise ValueError('Unknown stage')
        with self.lock:
            if self.mode != 'idle' or self.stop_pending:
                raise Task2Error('Movement or stage is active')
            self.require_fresh()
            self.cancel = threading.Event()
            self.mode = stage
            self.last_error = None
            self.stage_thread = threading.Thread(target=self.run_stage, args=(stage, self.cancel), daemon=True)
            self.stage_thread.start()
        return dict(stage=stage)

    def run_stage(self, stage, cancel):
        def report(kind, values):
            if kind == 'approach_sensor':
                sensor = values['sensor']
                self.event(f"O1 approach [{values.get('phase', 'unknown')}]: US={sensor['ultrasonic_cm']:.1f} cm valid={sensor['ultrasonic_valid']}")
                if sensor.get('raw_frame'):
                    self.event(f"STM32 raw: {sensor['raw_frame']}")
            elif kind != 'sensor':
                self.event(f'O1 {kind}: {json.dumps(values)}')
        self.event(f'O1 {stage} started; automatic approach and camera recognition')
        try:
            result = o1.main(['--execute', '--stage', stage, '--laptop-ip', self.laptop_ip],
                             shared_link=StageLink(self, cancel), cancel_event=cancel, event_callback=report)
            if result != 0:
                with self.lock:
                    self.last_error = 'O1 stage stopped; see event log'
            self.event(f'O1 {stage}: ' + ('completed' if result == 0 else 'stopped'))
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
            self.event(f'O1 failure: {exc}')
            try:
                self.stop()
            except Exception as stop_error:
                self.event(f'STOP failed: {stop_error}')
        finally:
            with self.lock:
                if self.mode != 'stop-unconfirmed':
                    self.mode = 'idle'
                self.cancel = None


def handler_for(controller):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, code, data):
            content = json.dumps(data, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if self.path == '/api/status':
                self.respond(200, controller.status())
            elif self.path == '/':
                content = Path(__file__).with_name('KC_robot_monitor.html').read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.respond(404, dict(error='Not found'))

        def do_POST(self):
            try:
                origin = self.headers.get('Origin')
                if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                    self.respond(403, dict(error='Origin rejected'))
                    return
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 4096:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(data, dict):
                    raise ValueError('Expected JSON object')
                if self.path == '/api/command':
                    result = controller.manual(data)
                elif self.path == '/api/stop':
                    result = controller.stop()
                elif self.path == '/api/stage':
                    result = controller.start_stage(data.get('stage'))
                else:
                    self.respond(404, dict(error='Not found'))
                    return
                self.respond(200, result)
            except (ValueError, Task2Error) as exc:
                self.respond(400, dict(error=str(exc)))
            except Exception as exc:
                controller.event(f'Request failed: {exc}')
                self.respond(500, dict(error=str(exc)))
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--http-port', type=int, default=8080)
    parser.add_argument('--laptop-ip', default='192.168.50.218')
    args = parser.parse_args()
    link = STM32Link(port=args.port)
    server = None
    controller = Controller(link, args.laptop_ip)
    try:
        # Bind before opening serial, so an occupied HTTP port cannot start motion.
        server = ThreadingHTTPServer((args.host, args.http_port), handler_for(controller))
        link.connect()
        link.stop()
        controller.event('STM32 connected. Manual controls ready.')
        print(f'Open http://192.168.50.5:{args.http_port} on your Mac.')
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print('Stopping monitor')
    finally:
        with controller.lock:
            if controller.cancel:
                controller.cancel.set()
        if link.ser is not None:
            try:
                controller.stop()
            except Exception as exc:
                print(f'STOP not confirmed: {exc}')
        if controller.stage_thread:
            controller.stage_thread.join(timeout=17)
        if server:
            server.server_close()
        link.disconnect()


if __name__ == '__main__':
    main()

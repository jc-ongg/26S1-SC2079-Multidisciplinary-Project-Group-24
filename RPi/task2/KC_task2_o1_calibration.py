#!/usr/bin/env python3
"""O1-only calibration. Prints a plan unless --execute is supplied.

Uses the v13 serial transport only; never invokes its route or STM32 TASK2.
Place O1/O2 with 60 cm clear gap. Commands are provisional calibration values.
"""
import argparse
import json
import math
import statistics
import subprocess
import time
import threading
from dataclasses import asdict
from pathlib import Path

from KC_stm32_link import STM32Link, Task2Error, VisionClient


def approach_o1(link, initial, record, check_cancel=lambda: None, target_cm=35, tolerance_cm=3):
    """Early-braked continuous approach, then bounded near-obstacle adjustment."""
    lower, upper = target_cm - tolerance_cm, target_cm + tolerance_cm
    def usable(sensor):
        if not sensor.ultrasonic_valid or not math.isfinite(sensor.ultrasonic_cm):
            raise Task2Error('Fresh valid ultrasonic required during approach')
        if sensor.ultrasonic_cm < 30:
            raise Task2Error('Approach crossed minimum stand-off')

    def settled(phase):
        seq, _ = link.sensor_snapshot()
        readings = []
        for _ in range(5):
            check_cancel()
            seq, sensor = link.wait_next_sensor(seq, timeout=1)
            record('approach_sensor', phase=phase, sensor=asdict(sensor))
            usable(sensor)
            readings.append(sensor.ultrasonic_cm)
        if max(readings) - min(readings) > 3:
            raise Task2Error('Stopped ultrasonic readings disagree by more than 3 cm; no further movement')
        distance = statistics.median(readings)
        record('approach_settled', phase=phase, distance_cm=distance, readings_cm=readings)
        return sensor, distance

    usable(initial)
    # Never authorise movement from the initial single reading.
    sensor, distance = settled('before_movement')
    if abs(distance - initial.ultrasonic_cm) > 3:
        raise Task2Error('Initial range disagrees with stationary readings; no movement')
    if distance > 65:
        # Speed-22 trial: request braking 28 cm before the 32 cm target.
        # Front echo validity is checked on every received packet.
        start_range = distance
        previous_range = distance
        cmd_id = link.send_motor_async('FWD', 22, 150)
        record('approach_command', cmd_id=cmd_id, phase='continuous', speed=22,
               distance_cm=150, stop_trigger_cm=60, from_range_cm=distance)
        seq, _ = link.sensor_snapshot()
        coarse_deadline = time.monotonic() + 20
        brake_range = None
        try:
            while True:
                check_cancel()
                if time.monotonic() >= coarse_deadline:
                    raise Task2Error('Continuous approach timed out')
                seq, sample = link.wait_next_sensor(seq, timeout=1)
                record('approach_sensor', phase='continuous', sensor=asdict(sample))
                usable(sample)
                if sample.ultrasonic_cm <= 60:
                    brake_range = sample.ultrasonic_cm
                    record('approach_brake', distance_cm=brake_range, trigger_cm=60)
                    break
                if sample.ultrasonic_cm > previous_range + 8:
                    raise Task2Error('Ultrasonic increased during continuous approach')
                if previous_range - sample.ultrasonic_cm > 15:
                    raise Task2Error('Ultrasonic dropped unexpectedly during continuous approach')
                previous_range = sample.ultrasonic_cm
                result = link.result_now(cmd_id)
                if result is not None:
                    raise Task2Error(f'Continuous approach ended before brake threshold: {result.status}')
        finally:
            link.stop(timeout=3)
        sensor, distance = settled('after_brake')
        record('approach_braking_result', trigger_range_cm=brake_range,
               stopped_range_cm=distance, additional_travel_cm=brake_range-distance)
        if distance > start_range or distance > brake_range + 3:
            raise Task2Error('Stopped range inconsistent with continuous approach')
    deadline = time.monotonic() + 120
    total_commanded = 0
    for attempt in range(40):
        check_cancel()
        if time.monotonic() > deadline:
            raise Task2Error('Bounded approach exceeded 120 seconds')
        if lower <= distance <= upper and lower <= sensor.ultrasonic_cm <= upper:
            record('approach_ready', distance_cm=distance)
            return sensor
        if distance < lower:
            raise Task2Error(f'Too close for trial turns; reposition to {target_cm} cm')
        # Every move lies in STM32's final 10 cm slowdown region.
        travel = min(10, max(1, math.floor(distance - target_cm)))
        total_commanded += travel
        if total_commanded > 150:
            raise Task2Error('Approach reached 150 cm total travel limit')
        check_cancel()
        cmd_id = link.send_motor_async('FWD', 16, travel)
        record('approach_command', cmd_id=cmd_id, phase='bounded', speed=16,
               distance_cm=travel, from_range_cm=distance)
        seq, _ = link.sensor_snapshot()
        move_deadline = time.monotonic() + 10
        while True:
            check_cancel()
            result = link.result_now(cmd_id)
            if result is not None:
                record('approach_result', **asdict(result))
                if result.status != 'DONE':
                    raise Task2Error(f'Bounded approach failed: {result.status}')
                break
            if time.monotonic() > move_deadline:
                raise Task2Error('Bounded approach movement timed out')
            seq, sample = link.wait_next_sensor(seq, timeout=1)
            record('approach_sensor', phase='moving', sensor=asdict(sample))
            usable(sample)
        previous_distance = distance
        sensor, distance = settled('after_movement')
        drop = previous_distance - distance
        # A larger echo must never authorise extra travel. Also reject implausible
        # decreases: the sensor may have switched from a background to O1.
        if drop < 0.5 or drop > travel + 4:
            raise Task2Error('Range change inconsistent with completed movement; no further movement')
    raise Task2Error(f'Bounded approach did not reach {lower}–{upper} cm in 40 movements')


def check_bypass_front(sensor, allow_missing_echo=False):
    if time.monotonic() - sensor.received_at > 1:
        raise Task2Error('Stale telemetry during bypass')
    if not sensor.ultrasonic_valid or not math.isfinite(sensor.ultrasonic_cm):
        if allow_missing_echo:
            return False
        raise Task2Error('Ultrasonic invalid during straight movement')
    if sensor.ultrasonic_cm <= 12:
        raise Task2Error('Front ultrasonic guard reached 12 cm')
    return True


def prepare_o2(link, vision, folder, record, check_cancel=lambda: None):
    """Experimental reverse-only preparation; never turns around O2."""
    def stable_range():
        seq, _ = link.sensor_snapshot()
        samples = []
        for _ in range(5):
            check_cancel()
            seq, sample = link.wait_next_sensor(seq, timeout=1)
            if not sample.ultrasonic_valid or not math.isfinite(sample.ultrasonic_cm):
                raise Task2Error('O2 preparation requires fresh valid front US')
            samples.append(sample.ultrasonic_cm)
            record('o2_sensor', sensor=asdict(sample))
        if max(samples)-min(samples)>3 or min(samples)<=12:
            raise Task2Error('O2 range unstable or too close; preparation stopped')
        return sample, statistics.median(samples)
    initial, distance = stable_range()
    folder.mkdir()
    side = recognise(vision, folder, record, check_cancel)
    initial, distance = stable_range()
    reverse_cm = min(15, max(0, math.floor(30-distance)))
    record('o2_prepare', side=side, front_range_cm=distance, target_cm=30,
           reverse_cm=reverse_cm, reverse_cap_cm=15,
           note='Rear clearance relies on tested O1 finish; front US cannot verify it')
    if reverse_cm:
        check_cancel()
        cmd_id = link.send_motor_async('REVS', 16, reverse_cm)
        record('command', cmd_id=cmd_id, command='REVS', speed=16, value=reverse_cm)
        seq, _ = link.sensor_snapshot()
        deadline = time.monotonic()+15
        try:
            while True:
                check_cancel()
                result = link.result_now(cmd_id)
                if result is not None:
                    record('result', **asdict(result))
                    if result.status != 'DONE':
                        raise Task2Error('O2 reverse did not complete')
                    break
                if time.monotonic()>deadline:
                    raise Task2Error('O2 reverse timed out')
                seq, sample = link.wait_next_sensor(seq, timeout=1)
                record('o2_sensor', sensor=asdict(sample))
                if (not sample.ultrasonic_valid or not math.isfinite(sample.ultrasonic_cm)
                        or sample.ultrasonic_cm < distance-3
                        or abs(sample.heading_deg-initial.heading_deg)>3):
                    raise Task2Error('O2 reverse range or heading guard failed')
        finally:
            link.stop(timeout=3)
        after, stopped = stable_range()
        if abs((stopped-distance)-reverse_cm)>4 or stopped-distance<0.5:
            raise Task2Error('O2 range increase disagrees with bounded reverse')
        if abs(after.heading_deg-initial.heading_deg)>3:
            raise Task2Error('O2 reverse changed heading excessively')
    else:
        stopped = distance
    record('o2_prepare_finish', side=side, front_range_cm=stopped,
           target_reached=stopped>=29, note='Stopped for inspection; no O2 turn')


def bypass_plan(side, stand_off=35, stage='bypass'):
    outward = 'TURNL' if side == 'left' else 'TURNR'
    inward = 'TURNR' if side == 'left' else 'TURNL'
    if stage in ('rough-bypass', 'rough-centre', 'o2-prepare'):
        # Empirical starting trial, not a scaled version of the 90-degree data.
        plan = [(outward, 50, 45), ('FWD', 50, 10),
                (inward, 50, 45), ('FWD', 50, round(stand_off))]
        if stage in ('rough-centre', 'o2-prepare'):
            # User-requested trial: omit the entire parallel straight segment.
            # Omit the command rather than sending FWD/0 (continuous movement).
            # Requires O2 absent; inward-turn clearance from O1 is unverified.
            plan.pop()
            plan += [(inward, 50, 45), ('FWD', 50, 16), (outward, 50, 45)]
        return plan
    sideways, final = (12, 21) if side == 'left' else (0, 16)
    # Keep the approximate finishing position when approach stand-off varies.
    final = round(final + stand_off - 35)
    plan = [(outward, 50, 90)]
    if stage == 'first-turn':
        return plan
    # FWD/0 is continuous movement in STM32, so omit the zero-distance leg.
    if sideways:
        plan.append(('FWD', 28, sideways))
    plan.append((inward, 50, 90))
    if stage == 'second-turn':
        return plan
    plan.append(('FWD', 22, final))
    return plan


def recognise(vision, folder, record, check_cancel=lambda: None):
    previous = None
    for attempt in range(4):
        check_cancel()
        image = folder / f'o1_{attempt}.jpg'
        subprocess.run(['rpicam-still', '-t', '500', '-o', str(image),
                        '--width', '640', '--height', '480', '-n'],
                       check=True, timeout=12, capture_output=True)
        arrow, confidence = vision._query('algo', vision.backends[0][1], image)
        check_cancel()
        record('vision', image=str(image), arrow=arrow, confidence=confidence)
        print(f'Camera: {arrow}, confidence={confidence:.3f}')
        accepted = arrow if arrow in ('left', 'right') and confidence >= 0.85 else None
        if accepted is not None and accepted == previous:
            return accepted
        previous = accepted
    raise Task2Error('No two consecutive confident arrow detections; bypass cancelled')


def main(argv=None, shared_link=None, cancel_event=None, event_callback=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['auto', 'left', 'right'], default='auto',
                        help='auto: approach + camera + bypass; left/right: manual 35 cm calibration')
    parser.add_argument('--laptop-ip', default='192.168.50.218')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--stage', choices=['first-turn', 'second-turn', 'bypass', 'rough-bypass', 'rough-centre', 'o2-prepare'],
                        default='bypass', help='Each stage starts from the original position, not the previous stage finish')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    print('O1 ONLY; auto mode starts centred in carpark facing O1.')
    print('Auto: ultrasonic approach -> stop -> two fresh confident arrow images -> bypass.')
    print('O2 must be at minimum 60 cm clear gap for calibration.')
    for side in (('left', 'right') if args.side == 'auto' else (args.side,)):
        print(side.upper(), bypass_plan(side, stage=args.stage))
    print(f'Stage: {args.stage}; STOP after selected stage. No O2 manoeuvre or parking.')
    print('Reset to the original start before each stage. Distances require physical calibration.')
    if not args.execute:
        print('Plan only. Supply --execute on the Pi for physical testing.')
        return 0

    link = shared_link if shared_link is not None else STM32Link(port=args.port)
    def check_cancel():
        if cancel_event is not None and cancel_event.is_set():
            raise Task2Error('Stage cancelled by STOP')
    vision_stop = threading.Event()
    vision_done = threading.Event()
    vision_thread = None
    vision_result = {}
    record_lock = threading.Lock()
    path = Path(f'KC_o1_{args.side}_{time.time_ns()}.jsonl')
    with path.open('w') as log:
        def record(kind, **values):
            with record_lock:
                if log.closed:
                    return
                log.write(json.dumps(dict(event=kind, monotonic=time.monotonic(), **values)) + '\n')
                log.flush()
                if event_callback:
                    event_callback(kind, values)

        def fresh():
            check_cancel()
            seq, _ = link.sensor_snapshot()
            return link.wait_next_sensor(seq, timeout=1.0)

        try:
            vision = VisionClient(laptop_ip=args.laptop_ip, timeout=3) if args.side == 'auto' else None
            if args.stage == 'o2-prepare' and vision is None:
                raise Task2Error('O2 preparation requires automatic arrow recognition')
            if vision:
                import requests
                response = requests.get(f'http://{args.laptop_ip}:8000/status', timeout=3)
                response.raise_for_status()
                if response.json().get('status') != 'ok':
                    raise Task2Error('Vision server status is not ok')
            check_cancel()
            if shared_link is None:
                link.connect()
            link.stop()
            seq, initial = fresh()
            record('start', sensor=asdict(initial))
            if vision:
                folder = path.with_suffix('')
                folder.mkdir()
                def vision_cancel():
                    check_cancel()
                    if vision_stop.is_set():
                        raise Task2Error('Vision worker cancelled')
                def scan_during_approach():
                    try:
                        while True:
                            vision_cancel()
                            _, sample = link.sensor_snapshot()
                            if (sample is not None and time.monotonic() - sample.received_at < 1
                                    and sample.ultrasonic_valid
                                    and 30 <= sample.ultrasonic_cm <= 65):
                                break
                            vision_stop.wait(0.1)
                        record('vision_parallel_start')
                        vision_result['side'] = recognise(vision, folder, record, vision_cancel)
                    except Exception as exc:
                        vision_result['error'] = str(exc)
                    finally:
                        vision_done.set()
                vision_thread = threading.Thread(target=scan_during_approach, daemon=True)
                vision_thread.start()
            closer_trial = args.stage in ('rough-centre', 'o2-prepare')
            lower, upper = (31, 33) if closer_trial else (32, 38)
            if args.side == 'auto':
                if closer_trial:
                    record('approach_target', target_cm=32, accepted_cm=[31, 33])
                    initial = approach_o1(link, initial, record, check_cancel, target_cm=32, tolerance_cm=1)
                else:
                    initial = approach_o1(link, initial, record, check_cancel)
            if not initial.ultrasonic_valid or not lower <= initial.ultrasonic_cm <= upper:
                raise Task2Error(f'Stopped stand-off must be {lower}–{upper} cm; adjust approach calibration.')
            side = args.side
            if vision:
                if not vision_done.is_set():
                    record('vision_wait_at_standoff')
                while not vision_done.wait(0.1):
                    check_cancel()
                check_cancel()
                side = vision_result.get('side')
                if side is None:
                    record('vision_stationary_retry', reason=vision_result.get('error'))
                    retry_folder = folder / 'stationary'
                    retry_folder.mkdir()
                    side = recognise(vision, retry_folder, record, check_cancel)
                else:
                    record('vision_parallel_ready', side=side)
                # Recheck fresh stand-off after vision, before allowing turns.
                seq, initial = fresh()
                if not initial.ultrasonic_valid or not lower <= initial.ultrasonic_cm <= upper:
                    raise Task2Error('Stand-off changed or invalid after recognition')
            plan = bypass_plan(side, initial.ultrasonic_cm, args.stage)
            baseline = initial.heading_deg
            expected_heading = baseline
            record('bypass_start', side=side, sensor=asdict(initial), plan=plan)
            for index, (command, speed, value) in enumerate(plan):
                check_cancel()
                if command == 'TURNL':
                    expected_heading += value
                elif command == 'TURNR':
                    expected_heading -= value
                cmd_id = link.send_motor_async(command, speed, value)
                record('command', cmd_id=cmd_id, command=command, speed=speed, value=value)
                deadline = time.monotonic() + 20
                diagonal = (args.stage in ('rough-bypass', 'rough-centre', 'o2-prepare')
                            and command == 'FWD' and value <= 20
                            and abs(abs(expected_heading - baseline) - 45) < 1)
                missing_echo_logged = False
                while True:
                    check_cancel()
                    result = link.result_now(cmd_id)
                    if result:
                        record('result', **asdict(result))
                        if result.status != 'DONE':
                            raise Task2Error(f'Command failed: {result}')
                        break
                    if time.monotonic() >= deadline:
                        raise Task2Error('Movement deadline exceeded')
                    seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
                    record('sensor', sensor=asdict(sensor))
                    # Front ranging cannot protect the sides/corners during turns.
                    if command == 'FWD':
                        echo_available = check_bypass_front(sensor, allow_missing_echo=diagonal)
                        if not echo_available and not missing_echo_logged:
                            record('diagonal_missing_echo', index=index, distance_cm=value,
                                   note='Continuing bounded diagonal; invalid US value ignored')
                            missing_echo_logged = True
                seq, after = fresh()
                record('after', index=index, sensor=asdict(after))
                if abs(after.heading_deg - expected_heading) > 5:
                    raise Task2Error('Heading differs from expected by more than 5 degrees')
            link.stop()
            if args.stage == 'o2-prepare':
                prepare_o2(link, vision, folder / 'o2', record, check_cancel)
            seq, after = fresh()
            record('finish', sensor=asdict(after))
            print(f'Stopped after stage {args.stage}. Measure corner clearance, centre position, and O2 distance. Log: {path}')
            return 0
        except (Exception, KeyboardInterrupt) as exc:
            record('failure', reason=str(exc))
            print(f'Test stopped: {exc}')
            return 1
        finally:
            vision_stop.set()
            try:
                link.stop(timeout=3)
            except Exception as exc:
                print(f'STOP could not be confirmed: {exc}')
            if vision_thread is not None:
                vision_thread.join(timeout=17)
            if shared_link is None:
                link.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())

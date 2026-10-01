"""Print calibration frames only. Never imports serial or transmits anything."""
import argparse


def frames(speed):
    if speed not in (25, 50):
        raise ValueError('Choose the measured speed 25 or current baseline 50')
    cases = [('FWD', distance) for distance in (10, 20, 40, 80, 120)]
    cases += [('REVS', distance) for distance in (10, 20, 40, 80)]
    cases += [(turn, 90) for turn in ('TURNL', 'TURNR', 'REVL', 'REVR') for _ in range(3)]
    return [f'{index}/MOTOR/{command}/{speed}/{value};'
            for index, (command, value) in enumerate(cases, start=1)]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', required=True, type=int, choices=(25, 50))
    args = parser.parse_args()
    print('PLAN ONLY. Nothing sent. Stop all controllers before hardware tests.')
    print('Use fresh unique IDs; await matching DONE after EACH command. Stop on error/timeout.')
    print('\n'.join(frames(args.speed)))

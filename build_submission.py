#!/usr/bin/env python3
"""Build the course's source-only TXT submission from this repository."""
from __future__ import annotations

import argparse
import ast
import hashlib
import os
from pathlib import Path
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent
NAME = 'MDP_2026-27S1-TEAM-24_Source_Code'
EXTENSIONS = {'.py', '.c', '.h', '.inc', '.kt', '.java', '.sh'}
FIRMWARE_APPLICATION = {'Core/Src/main.c', 'Core/Inc/main.h',
                        'Core/Src/ir_sensor.c', 'Core/Inc/ir_sensor.h'}
EXCLUDED_DIRS = {'venv', '__pycache__', 'build', 'node_modules', 'captures',
                 'own_results', 'runs', 'models', 'Debug', 'Release',
                 'Drivers', 'Middlewares', 'PeripheralDriver'}


def is_application(path: Path) -> bool:
    rel = path.relative_to(ROOT)
    if path.suffix not in EXTENSIONS:
        return False
    if any(part.startswith('.') or part in EXCLUDED_DIRS for part in rel.parts):
        return False
    if rel.parts[0] == 'Android':
        return 'src' in rel.parts and path.suffix in {'.kt', '.java'} and path.stem not in {
            'ExampleUnitTest', 'ExampleInstrumentedTest'}
    if rel.parts[0] == 'STM32':
        firmware_rel = Path(*rel.parts[2:]).as_posix()
        return (firmware_rel in FIRMWARE_APPLICATION or
                firmware_rel.startswith('Core/Inc/kc_') or
                firmware_rel == 'Core/Src/kc_task2_platform.inc' or
                (len(rel.parts) == 3 and path.name.startswith('test_') and path.suffix == '.c'))
    return rel.parts[0] in {'Algo', 'RPi'}


def source_files():
    for component in ['Algo', 'STM32', 'RPi', 'Android']:
        for folder, dirs, names in os.walk(ROOT / component):
            # Do not traverse virtualenvs or generated build trees. They can be
            # very large and can change while another tool is building.
            dirs[:] = [d for d in dirs if not d.startswith('.') and d not in EXCLUDED_DIRS]
            for name in names:
                path = Path(folder) / name
                if is_application(path):
                    yield path


def build(output: Path) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    archive = output / f'{NAME}.zip'
    sources = sorted(source_files())
    if not sources:
        raise RuntimeError('No application source found')
    files = {}
    manifest = ['SC2079 GROUP 24 - SUBMITTED SOURCE CHECKSUMS', '']
    for path in sources:
        rel = path.relative_to(ROOT).as_posix()
        data = path.read_bytes()
        if path.suffix == '.py':
            ast.parse(data, filename=rel)
        files[rel + '.txt'] = data
        manifest.append(f'{hashlib.sha256(data).hexdigest()}  {rel}.txt')
    files['README.txt'] = (
        'SC2079 MDP - GROUP 24 - SOURCE CODE SUBMISSION\n\n'
        'Algo: Task 1 path planner and Task 1/Task 2 recognition services.\n'
        'STM32/Task1 and STM32/Task2: the two firmware application versions.\n'
        'RPi/Task1 and RPi/Task2: task-specific controllers and communication.\n'
        'Android: one application shared by both tasks.\n\n'
        'Every source file retains its original extension followed by .txt.\n'
        'Contents match the current repository source. SOURCE_MANIFEST.txt\n'
        'contains SHA-256 checksums. Build files, configurations, resources,\n'
        'libraries, model weights and compiled output are excluded, following\n'
        'the source submission instructions. Existing notices are retained.\n'
        'The original repository is required to build and run the project.\n'
    ).encode()
    files['SOURCE_MANIFEST.txt'] = ('\n'.join(manifest) + '\n').encode()
    with tempfile.NamedTemporaryFile(dir=output, suffix='.zip', delete=False) as tmp:
        candidate = Path(tmp.name)
    try:
        with zipfile.ZipFile(candidate, 'w', compression=zipfile.ZIP_DEFLATED) as z:
            for rel, data in sorted(files.items()):
                entry = zipfile.ZipInfo(f'{NAME}/{rel}', date_time=(2026, 10, 19, 0, 0, 0))
                entry.compress_type = zipfile.ZIP_DEFLATED
                z.writestr(entry, data)
        with zipfile.ZipFile(candidate) as z:
            assert z.testzip() is None
            assert len(z.namelist()) == len(files)
            for rel, data in files.items():
                assert z.read(f'{NAME}/{rel}') == data
        candidate.replace(archive)
    finally:
        candidate.unlink(missing_ok=True)
    print(f'{archive}\n{len(sources)} source files; {len(files)} total TXT files; {archive.stat().st_size:,} bytes')
    return archive


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'output')
    build(parser.parse_args().output.resolve())

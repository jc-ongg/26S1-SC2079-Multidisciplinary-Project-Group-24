# Repository status — 2026-10-02

## Working branch and recovery

Later stash audit: [STASH_AUDIT_2026-10-02.md](STASH_AUDIT_2026-10-02.md) explains
the 28k saved paths and records the recovered reference documents. Source work
from both stashes was already integrated; old generated environments remain
in named local archive tags, not in the active project or GitHub uploads.
The user subsequently approved clearing the old list. Both verified active
stash entries were removed; complete snapshots remain in their local recovery
tags. No files on disk were deleted.

Use `main` for the integrated Android, Algo, laptop CV, RPi snapshot and STM32
sources. Existing component branches are compatibility/recovery names pointing
at the same integrated version, not different deployable projects. Future work
should use a focused `codex/<feature>` branch from main and merge back after tests.

This cleanup commits the existing turn-profile work, guarded Pi launcher and
documentation in separate logical commits. Retained branches are fast-forwarded
to the new main, with normal pushes only. It does not squash shared history,
force-push, delete branches, erase the saved stash contents, or remove generated
files/environments from disk. The old active stash entries were cleared only
after named recovery tags and useful-source reconciliation were verified.

Recovery refs:

- Previous integration head: `62e253dd1c145628e580f56cace93251a954720d`.
- Checkpoint tag: `archive/2026-10-02/before-repository-tidy`.
- Older component tips: `archive/2026-10-01/*` annotated tags.
- Both old stash contents remain in local `archive/2026-10-02/stash-*` tags;
  their active stash entries were removed with approval. Do not restore whole
  snapshots containing old environments and thousands of generated files.
- Ignored local backup: `output/repo-tidy-20261002/pre-cleanup.patch` and
  `working-files-before-cleanup.zip`. ZIP SHA256:
  `681af9875dc8fa795cccaceb7cbd8106384be8eb6bd84c06573c7abec2001ed2`.

## Pi access and source boundary

The user no longer has access to the physical Pi. No more SSH attempts,
deployments, process cleanup or hardware tests are part of this cleanup.
`RPi/` contains recovered source and selected follow-ups, not a guaranteed
up-to-date live mirror. Actual last-known source was `/home/nat/SC2079-MDP/rpi`.
Git commits and pushes do not change the physical Pi.

A source-only Pi backup remains at ignored
`output/repo-cleanup-20261001/rpi-source.tar.gz`, SHA256
`e4e1b1833374645b1007e8e7846b5fa544a80b8d2cb5528bd640b95cec04ba8d`.
It is not a full SD-card backup, OS configuration, firmware or photo archive.
When access returns, back up and compare actual files before any narrow deployment.

## Calibration and services

Default base movement profile is `original`, speed50, both overlays `original`.
Straight conversion remains10cm/cell. New speed50 overlays are independently
selectable, retain directed route costs/caches, and are provisional single-trial
turn measurements. Before/after straights are NOT a measured collision-safe arc.

| Overlay | Measured forward/back / sideways cm | Grid target cells | Extra straight before / after cm |
| --- | --- | --- | --- |
| Forward-left | 15 / 33 | 2 / 4 | 5 / 7 forward |
| Forward-right | 25 / 37.5 | 3 / 4 | 5 / 3 forward |
| Reverse-left | 29.5 / 15 | 3 / 2 | 1 / 5 reverse |
| Reverse-right | 34 / 21.5 | 4 / 3 | 6 / 8 reverse |

The forward table labels50 but its command examples label25; the implementation
uses50 as announced to the user. Bare90-degree turns are assumed and heading
measurements are absent. Reverse speed was explicitly corrected to50 by the user.

The rejected `measured25` experiment is preserved only as an explicit option;
it cannot be mixed with speed50 overlays. The Pi's `--calibrated` option selects
that old speed25 trial, not the speed50 overlays. See [calibration status](../Algo/CALIBRATION.md).

Algo8000 and temporary CV8001 run on the laptop. Original ONNX inference is
still unverified/missing in the last checked setup; temporary tracked `Algo/best.pt`
is not equivalent to the missing original model. This tidy-up does not start,
stop or restart any server, controller, camera or motor.

## Known unresolved issues

- Controller shutdown does not terminate/join all workers. Two independent
  Task1 parents, a missing motor ACK and garbled telemetry were observed after
  SSH drops/restarts. This cleanup does not fix or deploy shutdown behaviour.
- Pi intermediate pose queue is indexed by unadjusted planner commands despite
  inserted/cancelled movement padding; intermediate tablet poses/retry starts
  can be wrong. New-run state reset also needs separate verification.
- Physical turn shape, steering onset, heading and swept-arc/padding obstacle
  clearance are not verified by endpoint/unit tests.
- Task2 variants and original ONNX recognition are not certified by this cleanup.

## Offline verification

The curated Pi test runner blocks serial construction. Algo is checked in
separate processes for original baseline, rejected speed25 reference, forward50
only, reverse50 only and combined50 settings. The original golden maps must
remain unchanged with overlays off. Laptop launcher tests use mocked processes.
None of these tests initiates a physical run.

For the full historical discussion, see
[conversation handover](CONVERSATION_HANDOVER_2026-10-01.md). Its runtime/commit
snapshots must not override this newer access correction or live reinspection.

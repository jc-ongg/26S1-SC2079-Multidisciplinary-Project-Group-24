# Repository consolidation — 2026-10-01

**Current setup:** [speed50 / 5 cm test configuration and Pi cleanup](TASK1_GRID5_TEST.md)
supersedes the deployment/access status and source locations below. This document
preserves the original consolidation history and recovery tags.

Updated repository status on **2026-10-02**: see
[current status and cleanup](REPO_STATUS_2026-10-02.md). The Pi is no longer
accessible; runtime/deployment observations below describe the older session.
The old stash entries were subsequently removed with user approval after their
complete contents were archived locally and missing references were committed.
Use named recovery tags, not historical stash indices; see
[stash audit](STASH_AUDIT_2026-10-02.md).

## Where to work

Use **main** for integrated Android, Algo, laptop CV and recovered core Pi code.
Create narrowly scoped feature branches from main; review diffs and merge back.
Do not switch GitHub Desktop branches while services depend on files in the
checkout. A branch switch may remove source, virtualenv files or image folders.

| Former branch | Integrated history |
| --- | --- |
| android | Historical Android work, already an ancestor of main |
| rpi/stm-protocol-and-a5 | Protocol/A5 history, merged without rewriting it |
| rpi/task1-services-and-fixes | Teammate's laptop services and reconstructed Pi source, merged |
| codex/repository-consolidation | Integration/recovery work, included in main |

All four former local and GitHub branches were removed on 2026-10-02 at the user's
request. Each was verified equal to main at commit `66f7cd39`, so no further merge
was needed and no unique commits were discarded. Only `main` remains. The final
shared tip is preserved locally and on GitHub in annotated tag
`archive/2026-10-02/merged-branches-before-removal`.

The former branch positions are preserved as annotated archive tags:

- `archive/2026-10-01/android-before-consolidation`
- `archive/2026-10-01/rpi-stm-protocol-and-a5-before-consolidation`
- `archive/2026-10-01/rpi-task1-services-and-fixes-before-consolidation`

Branches were initially advanced through their existing history without rewriting
commits, then removed after verification and archival. The old active stash entries
were also cleared after their full contents were preserved in local recovery tags.
Useful Android Save/Use and temporary CV source were integrated, and assessment
references were recovered into `docs/`. Restore specific files only after inspecting
the archive tags; whole stash snapshots contain thousands of generated files and
old environments. See the stash audit for the local-only recovery refs.

The root .gitignore excludes virtual environments, generated captures/logs,
build output and machine-local IDE settings. These were not erased from disk.
The existing tracked Algo/best.pt remains in Git; new weights are ignored.

On this laptop, repository-local `core.excludesFile` also points to the ignored
`.gitignore-local` file. These rules remain active when inspecting an old tag
whose committed .gitignore lacks them. This prevents the 19,283 CV-environment
files and generated output from reappearing as pending source changes. They
remain on disk. Other laptops use the committed rules on the current branches.

## Pi source of truth and backup

Actual source: `/home/nat/SC2079-MDP/rpi`, accessed via SSH as nat at 192.168.50.5.
It is not itself a Git repository. The core controller and communication files
were recovered from the Pi, not replaced with the teammate's reconstructed code.
Historical standalone experiments and all versioned Task 2 variants remain in
the source snapshot; only the core and selected Task 2 runners are mirrored here.

Source-only snapshot created before consolidation:

`/home/nat/mdp-backups/repo-consolidation-20261001-bzHGLl/rpi-source.tar.gz`

SHA-256:
`e4e1b1833374645b1007e8e7846b5fa544a80b8d2cb5528bd640b95cec04ba8d`

A matching local archive is under ignored
`output/repo-cleanup-20261001/rpi-source.tar.gz`. It contains source scripts,
not installed packages, OS configuration, firmware or captured images.
The Pi's date is offset from the laptop; use hashes and inspection, not timestamps,
to decide which source is newer.

## Deployment boundary

No robot program was started, stopped, or restarted during consolidation.
Task 1 and both laptop servers were already running when checked.
The later optional profile guard was narrowly deployed to `task1_main.py`, with
backup `/home/nat/mdp-backups/calibration-trial-20261001-DLA9Sp/task1_main.py`.
No other later repository-only deployment is claimed. Current Pi state is unknown.

Repository-only follow-ups needing a reviewed Pi deployment after you stop Task 1:

- Environment-variable settings (LAPTOP_IP and optional ports/serial path).
- Safe run_week8.sh wrapper instead of the old process-killing sensor launcher.
- Retry original CV HTTP 404 as a no-detection result.
- Updated offline test runner and calibration-plan printer.

Back up again immediately before deployment; verify current file hashes, compare
the diff against this snapshot, and apply only the selected changes. Do not
bulk overwrite the Pi, including its logs, diagnostics, firmware or newer edits.
Review `git diff` in this repository before committing any later Pi-to-Git sync.

## Verification and limitations

- Android debug build and local unit-test task completed; no APK was installed.
- Mocked Pi tests cover Bluetooth framing, manual controls, ACK gating,
  quiet telemetry, capture/retry serialization and no-detection responses.
- Algo/CV tests cover 10 cm scaling, original turn constants, temporary-model
  label mapping, image errors and output-folder recovery.
- Both recorded map payloads produce the expected STM command lists offline.
- The live laptop health endpoints answered on ports 8000/8001. CV reported
  temporary best.pt. This does not verify a fresh physical camera capture.
- The Pi itself successfully reached both laptop health endpoints over Wi-Fi
  at 192.168.50.181 during the final check; no motor commands were sent.
- A test import triggered Ultralytics to install optional pi-heif 1.4.0 in the
  local ignored CV environment. Algo no longer imports the ML stack just for
  path planning, and regression tests disable Ultralytics auto-installation.
- Original ONNX model inference and Task 2 runners are not validated here.

Known controller follow-ups: the legacy path queue is indexed by unadjusted
planner steps even though adjusted STM commands can add/remove moves. Intermediate
tablet pose updates and automatic failed-obstacle retry start poses can therefore
be inaccurate. The controller also needs a separately tested new-run reset/network
failure pass. These are not solved by changing the distance multiplier; avoid
relying on automatic retries as a calibration measurement. Stop/restart Task 1
yourself between controlled tests, and inspect the command log.

The 10 cm grid conversion is approved, but the retained turns are a baseline,
not newly validated calibration. The rejected speed25 profile remains preserved
as an opt-in experiment; the newer speed50 forward/reverse overlays are also
provisional. Neither changes source defaults. See [calibration](../Algo/CALIBRATION.md).

# Teammate Task 2 monitor snapshot

The KC monitor and its helpers were recovered from the stopped physical Pi on
2026-10-07. Run `python3 task2/KC_robot_monitor.py` from the RPi root for that
separate monitor. It owns STM serial and must be stopped before Task 1 starts.
Motion/calibration logic is preserved unchanged; it was not validated for Task 2
by the Task 1 cleanup. Older numbered runners are under `../archive/task2/` and
in the Pi before-source backup. Their numbers do not establish team approval.

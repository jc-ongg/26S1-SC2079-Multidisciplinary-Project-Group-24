# Task 1 photo retry and partial-finish fix - 2026-10-07

The current Pi takes **one initial photo plus one retry per visit**. It accepts
the first non-NA result and stops taking pictures for that visit. One automatic
revisit pass remains for failed obstacles, so a revisited obstacle can have up
to four photos total. Camera/API errors end the current visit's attempts early.

The user's run failed to recognize obstacle 7, then its revisit request returned
no SNAP targets. The old controller raised PLAN ERROR and latched a motor fault,
leaving the robot stationary without the tablet's `finished` status.

After the final confirmed motor completion, the controller now ends normally
with partial results when the revisit is unavailable or fails. It also finishes
when targets remain unrecognized after the one revisit pass. It logs recognized
counts and missing obstacle IDs, clears dispatch, sends the tablet `finished`
(which stops its existing timer), and requests the current-run montage.
Initial map/route preparation errors still block Start. Actual motor faults or
missing ACKs still pause dispatch and do not claim an automatic finish.

The 180-second planning timeout, 10 cm map, speed50 and 30 cm turn clearance are
unchanged. This does not add an overall six-minute cutoff or alter the tablet's
manual Stop behavior for an in-flight motor command.

64 curated offline tests passed on the Pi, including empty revisit routes,
network failures, failure after both visits (four photos total), stale-map
responses, missing motor ACKs and real Ctrl-C/worker/Manager shutdown with
hardware mocked. The controller and one test file were backed up and installed
atomically while Task 1 was stopped. No motor commands were issued during this
update. Android and laptop service changes are not required.

Pi backup: `/home/nat/mdp-backups/task1-photo-stop-2026-10-07/`, containing original
files/hashes, staged checks, test log and deployment result. Local records are
under `output/task1-photo-stop-2026-10-07/`. The controller is left stopped for
restart with `python3 -u task1_main.py`; reconnect Android and resend the map.

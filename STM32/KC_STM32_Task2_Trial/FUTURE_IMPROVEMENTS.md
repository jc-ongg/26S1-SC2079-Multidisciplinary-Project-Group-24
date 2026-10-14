# Future improvements agreed with Kia Chun — 2026-10-14

Discussion only: do not implement, rebuild, flash, or push these without a later request. Current firmware remains the reference.

## Priority: faster O2 IR search

The native search is already continuous, not repeated2cm steps. Explore separate faster search and slower edge-confirmation speeds. On the first fresh clear reading reduce speed; require two fresh consecutive clear readings before braking/turning. Investigate faster reverse object acquisition with two fresh object readings. Verify actual sensor acquisition cadence and whether the adapter can change speed within a command before choosing implementation. Retain travel/time bounds, proximity checks, STOP/heartbeat behavior and actual encoder travel including braking. Evaluate confirmation delay plus braking distance against the side50cm boundary; faster motion must not recreate overshoot. Physically validate before making it the default.

## Separate trial: smoother O1 bypass

Explore rolling turn/straight steering transitions with controlled speed instead of a full stop at every segment. Existing speed50 turn translations were calibrated as separate stopped commands; a continuous version changes swept clearance and endpoint and requires new physical calibration. Keep the present route as rollback/reference. RIGHT finishes with a reverse turn, which still requires a stopped direction change. Do not simultaneously alter O1 geometry and O2 search while diagnosing a new trial.

## Other candidates

Approach deceleration to avoid forward overshoot/reverse correction; reuse fresh verified stationary range across unchanged-pose handovers; accelerate/cruise/decelerate long return travel. Persistent camera session is a separate Pi improvement. Latest O1 acceptance compensation needs physical validation before further performance changes.

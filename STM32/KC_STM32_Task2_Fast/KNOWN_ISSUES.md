# Current status — updated2026-10-14

Latest shared firmware: Release/KC_Task2_KiaChun_Long_Return_Fix.hex, startup RETURN_LANE_TRIAL/2/22. Source includes heading-feedback correction, early wide-lane correction, and the fix for the40cm parking-preflight search restriction. Use the Protocol2 bridge included in pi/.

Kia Chun reports the latest runs have been "not too bad". The supplied17-attempt transcript contains13 COMPLETE,3 ERROR_PARK_DEPTH_UNCONFIRMED and1 ERROR_STRAIGHT_HEADING. Completed attempts cover all four arrow combinations. See analysis/physical_runs_2026_10_14 for the complete transcript and per-run outcomes. Firmware COMPLETE is not independent evidence of collision-free physical clearance.

## Remaining issues

- Parking can still stop on unreliable final US confirmation: runs638492705,967884979,127117332.
- Run431109947 stopped with ERROR_STRAIGHT_HEADING during the final parking approach. Exact cause needs sensor telemetry; no new guard changes are included in this publication.
- Turn translations use measured endpoints. Intermediate turn sweeps are modelled with two illustrative fits, not measured trajectories. Pose is estimated when O1 IR cannot be acquired.
- Earlier outward O1 bypass could finish very close to O2 (recordedUS2.90cm); this return update does not change O1 bypass.
- Model/API code and weights on the laptop are separate. This update keeps the existing image API contract and protocol2 bridge. A separately installed Protocol4 bridge is not compatible with this firmware.

Old named HEXs are retained for rollback. Prior collision analyses remain in analysis/parking_collision. Latest return details are in analysis/friend_Task2_return. This branch remains a draft for review; no merge or board flash is performed by publication.

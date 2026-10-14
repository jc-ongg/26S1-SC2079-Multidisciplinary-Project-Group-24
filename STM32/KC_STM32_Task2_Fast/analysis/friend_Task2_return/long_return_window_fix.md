# Long return planning fix — run428994270

Reported layout: carpark opening toO1 front60cm; O1 toO2 gap150cm.
Firmware identity and recognition succeeded. O1LEFT79.1%, O2LEFT84.1%.
After bypassingO2, estimated poseX51.40,Y231.92; after forward19cm
(actual19.58cm), X51.40,Y212.35. Then STATE27 -> ERROR_RETURN_LANE_PATH.
No parking diagonal was issued.

Root cause: new lane-correction preflight asked kce_choose to prove a later
parking entry, but that function only searched from currentY-40. With a long
obstacle gap, the corrected lane endpoint is still much farther than40cm
from the parking-entry region. This was an artificial search bound, not a
US spike, camera failure, or evidence of an obstructed straight return.

Fix: anchor the bounded parking-entry search to O1-front +/-100cm rather than
currentY-40. Still constrain it to already traversed lane positions, and sample
the entire connecting straight using kce_reposition_clear. Reject paths crossing
O1 or park walls. No sensor thresholds were tightened or new motion types added.
This allows the early correction and subsequent long straight towardO1.

Regression reproduces the exact failed pose/anchors (O1front72.95,
O2front231.50), verifies the final-entry connection exceeds40cm and is clear,
and verifies the same connection atX0 is blocked byO1. Four arrow combinations
also complete host simulation at both60cm and150cm obstacle gaps. ASan/UBSan
and ARM Release build pass. Physical parking remains to be tested.

Flash KC_Task2_KiaChun_Long_Return_Fix.hex. Startup marker:
!KC_T2/RETURN_LANE_TRIAL/2.00/22.00;
Protocol2 Pi bridge and camera API unchanged. Pre-change source and HEX are
preserved in backup_long_return_window. No robot movement/flash performed.

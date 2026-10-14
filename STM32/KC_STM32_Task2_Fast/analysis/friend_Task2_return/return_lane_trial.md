# Return lane correction trial — 2026-10-14

Status: built and host-tested; NOT physically validated. No flash or robot motion performed.

Reference: Task2 branch commit9365beb33e9db56b3624c84643743085eaea793e,
`RPi/box_inspect_task.py`. See box_inspect_task_snapshot.py. The friend uses saved
outbound front range and an estimated O2 span to plan the return, with a tuned
45-degree / 30-degree turn pair. This trial borrows early return planning and
shorter entry, not that robot's exact constants or speed100 diagonal.

Our logged return offsets reached56.11cm; a58cm diagonal plus two turns was
about77.8cm of longitudinal movement. A front US check after that movement
could not prevent striking the park side first. Heading_Feedback_Fix also
corrected stale gyro-angle double counting, a separate cause of false stops.

New path:
1. After the final O2 turn, advance at22 until estimated centreY is below
   O2-front minus18.5cm. This uses the previously verified outbound US anchor.
2. If the return lane is wider than45cm, preflight a45/45 S correction between
   obstacles. TargetX is+30cm on the positive side, -26cm on the negative side,
   accounting for measured asymmetric turns. Require a5cm feasible staging
   interval AND a feasible subsequent parking entry. If none exists, stop.
3. Reposition at12, check the actual settled encoder endpoint, then execute
   turns at50 and diagonal at22. Retain2-degree heading and2cm diagonal-overrun
   guards. Confirm the estimated endpoint is within2.5cm of its target.
4. Find O1 using the existing IR landmark/fallback, then choose a final entry
   that fits the opening. A narrower lane keeps the final diagonal shorter.
   Use the existing front-US final approach and side-wall confirmation.

Example x56.11, O1front73.27, O2front143.17: after clearing toY124.41,
choose correction nearY121, diagonal15; estimated finishX30.33,Y73.83.
The final entry is approximately21cm diagonal rather than58cm. Candidate
positions vary with actual encoder travel and O1 IR reanchoring.

Park envelope:60cm wide,50cm deep. MouthY <=min(38.5,O1front-60+3), based on
the rules opening-to-O1 minimum60cm and a3cm range allowance. The sampled
geometry uses23x18.8cm robot body plus5cm margins and0/2cm diagonal coasting.
It checks both linear and tangent-matched cubic illustrative turn sweeps.
These fits are NOT measured swept trajectories and do not guarantee clearance.
Initial placement/US errors can still invalidate the assumed envelope.

O2 width is not treated as fully measured by the partial back-edge crossing.
No new width inference, Pi camera changes, or O1 bypass changes are included.
Turns remain calibrated45degrees; the friend's uncalibrated30-degree finish
is not copied. Protocol2 bridge remains required.

New states:27 RETURN_CLEAR,28 RETURN_TRIM_ALIGN,29 RETURN_TRIM.
New events:RETURN_LANE_TRIAL,RETURN_LANE_PLAN,RETURN_LANE_END.
Rollback: backup_friend_return contains the exact preceding geometry/FSM/tests,
README/checksums and KC_Task2_Heading_Feedback_Fix.hex.

Validation: ASan/UBSan host state-machine tests (four arrow combinations,
reported wide-lane shortening, absent-path refusal, actual encoder endpoints,
heading/overrun guards) and production adapter tests passed. ARM Release build
passed. This is a trial awaiting physical clearance testing, not a proven fix.

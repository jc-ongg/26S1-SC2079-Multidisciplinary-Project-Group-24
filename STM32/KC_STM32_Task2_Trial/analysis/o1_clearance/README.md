# O1-only clearance analysis — 2026-10-14

This is an offline study of the latest acceptance-compensated O1 routes. No firmware, HEX, Pi service or GitHub branch was changed by this analysis.

## Inputs and limitations

- O1 spans X=-5..5,Y=0..10cm. Minimum O1-rear to O2-front gap60cm puts O2 front atY=70cm, notY=60 or80.
- Robot outer footprint is assumed to be23cm long ×18.8cm wide, centred on the reported robot centre. This must include protruding wheels; unmeasured overhang is not covered. Exact rear axle/wheel locations are unavailable, so the whole rectangle is checked rather than claiming an exact wheel contact point.
- US assumed at front centre11.5cm ahead of robot centre. Accepted gap30–34cm gives starting centreY=-gap-11.5.
- Measured forward LEFT45 translation(side,forward)=(-13,13.25), RIGHT45=(15,18), reverse LEFT45=(-1.5,-12.5)cm. All bypass commands use50.
- LEFT: L45,F10,R45,F(round(9+gap-32)),R45,F10,L45.
- RIGHT: R45,F7,L45,F(round(20+gap-32)),L45,F26,reverse-L45.
- Two intermediate turn models share these measured endpoints: linear centre/yaw interpolation and a cubic Hermite fit with centre tangents aligned to signed vehicle heading. Neither is a calibrated steering/wheelbase trajectory. Agreement between them does not bound the real path; linear interpolation need not be physically realisable by this chassis.
- Sampling: no more than0.2cm chord increments or0.5degree heading increments. No continuous collision proof between samples. Rectangle/obstacle distance uses SAT intersection and Euclidean edge distance. Negative values mean model overlap, not a measured physical impact.
- O2 is conservatively treated as an infinitely wide front plane. Positive plane clearance is independent of O2 width under this model; negative clearance could be beside a finite-width object rather than an actual collision.

## Nominal results

| Route | Minimum O1 gap, linear / Hermite | Minimum O2-plane gap | Tightest O1 part |
|---|---|---|---|
| LEFT |2.40 /3.55cm|5.71cm|segment5, inward RIGHT45|
| RIGHT |4.14 /5.70cm|5.58cm|segment5, inward LEFT45|

For RIGHT the closest O2 approach occurs at the end of segment6 (26cm inward diagonal), BEFORE the final reverse turn. Its front-corner projection at45degrees is(11.5+9.4)/sqrt(2)=14.78cm; checking only a heading-zero front offset would overstate clearance.

Raw nominal endpoints: LEFT(+0.055,+52.788)cm; RIGHT(-3.480,+39.747)cm. Firmware later resetsX to0 and substitutes an empirical Y anchor. This does not physically centre the robot. RIGHT26cm was an empirical correction for a reported3cm right finish; the older turn endpoint model predicts3.48cm left instead. This discrepancy means the turn translation model and physical endpoint observations are not fully reconciled. Do not change RIGHT back to22 solely to force the provisional model endpoint tozero.

## Placement / straight-travel scenarios

360 sampled scenarios: both routes, gaps30/30.5/32/33.5/34cm, starting lateral offsets-2/0/+2cm, starting heading errors-2/0/+2degrees, both illustrative turn models, and either nominal straights or +1cm actual travel on EVERY forward straight. These are explicit sensitivity assumptions, not measured error bounds or probabilities. No additional independent turn displacement error is included.

- Worst LEFT O1 result:0.96cm model overlap (start2cm right, heading2degrees right, gap34, linear turn model).
- Worst RIGHT O1 result:0.50cm remaining.
- Worst O2-plane gaps across these scenarios: LEFT2.53cm, RIGHT2.28cm.

No claim that these exact impacts will occur; the findings show that current measurements cannot certify a robust clearance margin. An initial heading error also accumulates lateral displacement over the bypass.

## Why simply extending the lane is inadequate

Nominal cases only, minimum over the two illustrative path models:

| Route | Extra side-lane travel before inward turn | Minimum O1 gap | Minimum O2-plane gap |
|---|---:|---:|---:|
| LEFT | +0 cm | 2.40 cm | 5.71 cm |
| LEFT | +1 cm | 2.87 cm | 4.71 cm |
| LEFT | +2 cm | 3.33 cm | 3.71 cm |
| LEFT | +3 cm | 3.77 cm | 2.71 cm |
| RIGHT | +0 cm | 4.14 cm | 5.58 cm |
| RIGHT | +1 cm | 4.46 cm | 4.58 cm |
| RIGHT | +2 cm | 4.75 cm | 3.58 cm |
| RIGHT | +3 cm | 5.03 cm | 2.58 cm |

Each extra centimetre consumes approximately one centimetre of O2 clearance. Merely accepting a wider US range does not add lateral or heading observability.

## Recommendation

Keep current firmware as the physical-test reference. Do not claim guaranteed clearance or add lane length blindly. Verify outer footprint/axle reference and reconcile the45degree translations using the same robot reference point. Obtain centre position/heading at several points DURING each LEFT/RIGHT45 and reverse-LEFT45 turn (overhead video with floor scale can supply this later). Then fit a trajectory and re-evaluate with observed encoder overrun and placement errors. Existing stopped approach US can confirm the initial gap but cannot observe lateral centring; a post-bypass reverse does not protect the preceding diagonal from O2.

Near-term physical check: both routes at minimum60cm spacing, gap near30 and34, measure closest wheel-to-O1 clearance during segment5 and closest front/corner-to-O2 clearance through segment6/7. Accurate starting alignment matters. If these margins are insufficient, redesign lateral detour/finishing-turn geometry jointly rather than just lengthening the side lane. No new calibration values are silently substituted here.

## Reproduce

`python3 check_o1.py` regenerates `results.json`. `lane_tradeoff.json` records the separate lane-extension comparison. `nominal_paths.svg` illustrates the nominal Hermite paths; it is not a measured sweep.

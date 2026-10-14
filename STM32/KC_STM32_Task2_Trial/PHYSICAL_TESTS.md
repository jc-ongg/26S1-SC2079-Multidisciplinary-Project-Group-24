## Native O1 encoder-tolerance firmware — user test report

User reports two LEFT and two RIGHT O1 runs passed. Attached logs confirm all four reached COMPLETE and run-ID DONE without ERROR. Initial US 56.59–59.36 cm; stopped approach US 31.38–32.43 cm. RIGHT confidences 0.697 and 0.551; LEFT 0.912 and 0.944. The previously failing RIGHT 7 cm segments completed. Position estimates before the empirical finish anchor: RIGHT X -0.41/-0.47 cm, LEFT X +0.09/+0.10 cm. These X values are calculated, not measured; COMPLETE X=0 is an imposed calibration anchor. No numerical physical centreline or heading measurements supplied for these four runs. O2, return and parking remain unvalidated on this native firmware.

Source log: /Users/kiachun/.codex/attachments/1754a36b-8d3a-49fa-97ae-4ddabbf8ad55/Pasted text.txt


## O2 RIGHT back-face near-contact

Parallel-recognition run 945908571 (Pi log KC_native_task2_1787525664945913145.jsonl) stopped at back acquisition. Retrieved telemetry: LEFT IR repeatedly 7.7–7.8 cm; user confirms nearly touching. No threshold reduction. Trial correction adds RIGHT-only FWD 10 cm along the side before the second LEFT turn. Awaiting physical test.

## Second native RIGHT/RIGHT full run

User confirms parked nicely inside. Run 576775012 ended with ERROR_PARK_WALLS after US 12.61 cm; retrieved final left/right IR 39.4/18.3 cm sum57.7 above old upper57. Route physically parked, software completion rejected. Parking confirmation trial adds bounded stationary retry and upper sum allowance59, keeping both-wall requirements.

## Forward100 trial contact after O1

Run563489687 reported DONE but user observed physical contact. User clarified contact was to O2 immediately after O1 bypass, before O2 scanning/turning. O1 FWD7/14/8 encoder travel approx9.43/16.24/10.34 cm. Software completion is not a collision-free result. New Fast_Transit_O1_Calibrated trial restores O1 speed50/original slow final cap while retaining speed100 only for long approach/return/centring commands. Awaiting retest.

## Initial O1 contact despite full-run DONE

User clarified run348108482 rear wheel touched O1 during the initial bypass toward O2, not the return. Run otherwise reported parking completion at16.70cm. New O1 RIGHT trajectory trial adds6cm before inward turn and replaces final turn with measured reverse-LEFT45, compensating diagonal and endpoint anchor. The preceding run136990360 physically parked fully inside at5.54cm front gap but failed confirmation; one geometry-bounded reverse recovery added. Neither new behaviour physically tested yet.

## Five-run parking review and authorised corrections

Attachment03705e74-aadf-4fcb-8660-89bcb2e574ec: two RIGHT/RIGHT parking US failures (retrieved raw echoes80–500cm/invalid with stable side walls), one LEFT/RIGHT PARK_INSIDE11.59cm, LEFT/LEFT120cm-start stable11.94cm rejected against18.55cm expected, LEFT/LEFT149cm-start rejected90.17cm by80cm cutoff. User confirms first four physically inside, last outside. Current trial adds stationary confirmation-only revalidation, bounded long parking approach and measured O1 RIGHT centring compensation. No physical validation of these changes yet.

## Long approach / reverse-start encoder failure

Latest new run775486754 in attachmentd2118a80-49d0-4e53-bf3b-af4397319e4d is O1 LEFT. FWD94→96.12 actual, US30.18, REVS3→negative progress resulting poseY+2.53. Retrieved telemetryUS29.1→26.7→26.2 during reverse startup. Trial fixes straight completion with bounded encoder quiet interval and restores speed100 final10cm calibrated cap850. Includes all prior centring and parking changes. Awaiting physical test.

## Brake-settle physical logs

Attachmentbc1cfb56-87a0-47d5-b540-22bf71d87227: LEFT O1-only291142360 and RIGHT O1-only361475004 reportDONE with PROFILE26/850. No braking/encoder error in these runs. Physical RIGHT26cm endpoint measurement not supplied. Full attempts subsequently hit O1 min3cm oscillation, one startupNOT_READY, O2 vision failure, and O2 common-view oscillation despite LEFT arrow. Current Fine_Positioning trial reduces minimum correction2cm and uses cached O2 arrow target. Pending physical validation.


### Reported parking collision — run567849394
User reports contact during diagonal centring and again on carpark entry. O1 LEFT80.3%, O2 LEFT63.6%. O1 return landmark missed; estimated X56.11; old centring F58 at100; finalUS3.17,ERROR_PARK_CONFIRM. New Parking_Entry_Geometry_Trial is built but physical validation is pending. Earlier2.90cm O2 gap remains unresolved.


### Latest feedback after parking entry trial
Kia Chun reports carpark remains problematic. No latest trace or contact location supplied yet. Mark parking unresolved; do not treat simulated clearance checks as physical validation.

## Latest full-run feedback (2026-10-14)

Long_Return_Fix marker2: user supplied17 attempts,13 firmware completions covering all four arrow combinations. Three parking-depth confirmation stops and one final straight-heading stop remain. See analysis/physical_runs_2026_10_14 for the transcript and per-run summary. User reports overall behaviour is "not too bad"; no precise collision/parking-position measurements were supplied with this batch.

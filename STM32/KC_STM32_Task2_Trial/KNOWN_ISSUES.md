# Unresolved issues — updated2026-10-14

## Parking still problematic — latest physical feedback

Kia Chun reports: "the carpark still have an issue" after being given the parking entry geometry trial. No latest-run log, measured offset or exact contact location has been supplied with that report. Do not claim the latest failure has the same precise cause as the earlier collision. The shared branch intentionally remains a draft/work in progress.

## Earlier documented collision — run567849394

O1 LEFT and O2 LEFT recognised80.3%/63.6%. Return failed to acquire O1 IR and used estimatedX56.11. From old rear-clearY57.90 it ran RIGHT45, FWD58 at100, LEFT45. Actual diagonal58.38 plus calibrated turns advanced77.84cm toward parking. User reports contact during diagonal and again on entry; finalUS3.17 produced ERROR_PARK_CONFIRM. That final rejection happened after the reported contact.

The new trial plans an earlier start from the known return lane, checks complete sampled footprint/repositioning, rechecks actual positioning endpoint, and reduces diagonal to22. For that recorded case, model choice isY97.50 and approximately24cm reverse from the earlier landmark-search stopY73.61. **This mathematical replay does not prove the physical path fits**, and latest user feedback remains unresolved.

## Remaining uncertainty / reviewer focus

- Real intermediate45degree turn sweeps are unmeasured. The planner tests two illustrative fits sharing measured endpoints; these are not certified bounds.
- No O1 return landmark means lateral pose remains estimated. Resetting X0 after outward O1 bypass is a calibration assumption, not observed centring.
- Park wall envelope assumes60cm inside width,50cm depth, start fully inside aligned approximately with centreline. It does not measure this setup's wall position. Inflated body margin5cm does not establish actual pose uncertainty.
- Side-wall thickness assumed10cm. Reposition bounded to recorded return corridor, but actual surroundings/pose still need validation.
- Most recent documented run had US2.90cm immediately after outward O1 bypass, before reversing away from O2. Parking planner does not fix that earlier proximity risk.
- New model running on Kia Chun's laptop is a separate Task2 arrow API; model/API code and weights are not bundled in this STM32-only update. Existing Pi multipart8001/predict contract remains compatible. Success recognising arrows does not establish movement clearance.
- Protocol2 firmware/bridge in this folder differs from the separately installed Protocol4 bridge. Use matched pairs; keep identity checks.

Next review needs the latest physical-failure log and contact phase, then reconciliation of true turn sweep/park reference before modifying distance or confirmation thresholds. See analysis/parking_collision and analysis/o1_clearance for prior calculations. No merge or board flash performed by this publication.

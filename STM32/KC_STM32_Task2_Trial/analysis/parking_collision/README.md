# Physical parking collision report — run567849394

User reports contact during diagonal centring toward the carpark and again on entry. Do not treat the final software failure as evidence that earlier motion was collision-free. No firmware change made by this analysis.

Firmware identity: protocol2, O1_ACCEPTANCE30/34. Native bridge protocol2_compatible. O1 LEFT recognition80.3%; O2 LEFT63.6%. Both accepted early.

## Trace and cause

- O1 bypass ends then US2.90cm facing O2. Robot reverses25command /25.98actual, then forward2command /2.11actual. Accepted O2 LEFT26.09cm. The very small initial O2 gap is a separate unresolved bypass proximity risk; improved recognition does not resolve it.
- Return finishes O2 bypass at estimated X56.11,Y140.41. Initial return48command /48.75actual, then O1 landmark search30command interrupted after18.05actual.
- No O1 IR landmark acquired: O1_POSE_ESTIMATE56.11/73.61. Thus lateral X is provisional turn/encoder geometry, not an observed O1/carpark offset. At this offset the side IR may be outside its usable O1 range; actual IR data are needed to establish why acquisition failed.
- Rear-clear forward15command /15.71actual -> centre phase X56.11,Y57.90.
- Centring RIGHT45 -> X41.11,Y39.90, FWD58command /58.38actual at speed100 -> X-0.17,Y-1.38, LEFT45 -> X-0.35,Y-19.94.
- First parking US confirmation3.17cm; ERROR_PARK_CONFIRM. Current recovery requires at least4cm, so it correctly does not authorise a blind reverse at3.17. That final rejection cannot prevent the preceding collision.

The code chooses diagonal from lateral shift only:
shift=15+(13.25-13)/sqrt(2)=15.1768cm.
diagonal=(56.11-shift)*sqrt(2)=57.8883cm ->58cm.
Actual diagonal advances 41.2809cm longitudinally. First RIGHT45 advances18cm, final LEFT45 advances(13.25+13)/sqrt(2)=18.5616cm. Total toward carpark=77.8424cm.

KCT_CENTRE does not test carpark mouth/front-wall/side-wall swept geometry. Its only geometry check is diagonal0..70cm. Neither clipping that limit nor increasing parking acceptance safely fixes the missing entrance check. It sets US anchor unknown and enters KCT_PARK only AFTER the diagonal and both turns. No fresh wall check protects the earlier centring path. All non-diagonal movement and gyro/motion guards still exist, but these do not observe park wall clearance.

## Required redesign

Plan lateral and longitudinal displacement together, verify full footprint/swept turns against the actual park entrance and wall reference, and be aligned before entering its narrow corridor. A large-offset return may require a different, staged manoeuvre rather than the same forward S bend. Require a credible return pose; do not confuse resetting the internal X coordinate or an unobserved O1 landmark with measurement. After alignment, use slow front-US-controlled entry with fresh side-wall confirmation. Speed reduction limits impact/stopping distance but alone does not make a geometrically colliding path safe. Do not lengthen diagonal, weaken final depth threshold or simply truncate diagonal while claiming centring.

Physical park mouth position relative to start and actual return lateral offset are not established by this pasted log. Width60/depth50 are existing firmware assumptions; they are not fresh measurements of this setup. These must be distinguished before certifying a new route. Current known issue requires path redesign/testing, not another unqualified full run.

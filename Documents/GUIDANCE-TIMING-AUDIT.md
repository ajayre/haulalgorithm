# TheShop2 guidance timing audit

Date: 2026-07-15

## Result

The final 3,152-route GPS guidance contains 3,667,783.146 m (2,279.05 mi) of driven centerline. With the configured operating speeds and dynamics, it requires:

- **433.483 h of continuous driving**. This is the primary machine-motion time and the appropriate starting point for diesel planning.
- **541.854 h of scheduled window at 80% operational availability**. The additional 108.371 h is an explicit availability allowance, not turn time and not driving time.
- **481.648 h at 90% availability** or **619.261 h at 70% availability**.

The 80% factor must represent delays outside the generated GPS track, such as servicing, refueling, operator breaks, and other non-driving losses. It must not include turns, lineups, deadheading, or grading because those are already present in the 433.483 h guidance result.

The authoritative machine-readable report is [`guidance_timing_full_promoted.txt`](../assets/theshop2_guidance/guidance_timing_full_promoted.txt). The first-ten-route regression report is [`guidance_timing_routes_1_10_promoted.txt`](../assets/theshop2_guidance/guidance_timing_routes_1_10_promoted.txt).

## What the result includes

| Category | Distance | Driving time |
|---|---:|---:|
| Blade-down cut, fill, and grade work | 360.921 mi | 71.579 h |
| Lineups, exits, and fill-to-grade turns | 675.859 mi | 123.681 h |
| Transit, next-cut transit, and manual-assist guidance | 1,242.274 mi | 238.223 h |
| **Total** | **2,279.055 mi** | **433.483 h** |

The overall driven average is 5.257 mph. Loaded motion accounts for 2,061,242.1 m and 257.065 h; unloaded motion accounts for 1,606,541.0 m and 176.418 h. Transit is not assumed to be a straight full-speed line: the final path contains keyholes, loops, boundary detours, and curved connections. The main `Transit` phase alone is 946.769 mi and averages 4.699 mph.

## Timing algorithm

The estimator times the final driven centerline, not the original haul approximation and not a sum of independently stopped lifecycle segments.

1. Every lifecycle segment records whether the scraper is loaded. A route starts unloaded, becomes loaded after cutting begins, remains loaded through all fills, and becomes unloaded for grading and travel to the next cut.
2. Every nonzero polyline edge is assigned an operational speed cap:
   - cut 4 mph;
   - fill 5 mph;
   - grade 8 mph;
   - straight loaded blade-up travel 10 mph;
   - straight unloaded blade-up travel 13 mph.
3. Curvature is measured across phase boundaries using equal adjacent path distances. Collinear sliver points introduced when a connector is split between work and transit are combined before curvature is evaluated.
4. Turn speed is interpolated linearly from heading change. For the same evaluation distance, `severity = clamp(observed turn angle / minimum-radius turn angle, 0, 1)` and `speed = full payload speed - severity * (full payload speed - configured turn speed)`. Therefore zero turn angle permits full loaded or unloaded speed, while a turn at the 25 ft minimum radius uses 4 mph. Normalizing both angles over the same distance makes the result independent of waypoint density.
5. Cut, fill, and grade caps still apply while the blade is down. Blade-up lineups and exits are no longer automatically held at 4 mph: the continuous speed profile accelerates on a straight departure and brakes for the next work section or curve.
6. A forward/backward speed envelope enforces acceleration and braking continuously over the entire chained route. The default acceleration and deceleration are both 0.484293 m/s2, equivalent to 0-to-13 mph and 13-to-0 in 12 seconds.
7. The vehicle begins at rest at the start of the field plan and finishes at rest at the end. A phase or segment boundary does not cause a stop. A disconnected geometry boundary would force a stop and be reported; the final guidance has zero such boundaries.
8. Each speed interval is integrated as a triangular or trapezoidal motion profile. If its speed cap can be reached, time is acceleration plus cruise plus braking. Otherwise the reachable peak speed is solved from distance, entry speed, exit speed, acceleration, and deceleration.
9. Explicit blade or takeover events are added once per state transition. Their configured durations are currently zero because blade and operator transitions do not require the tractor to stop.
10. Operational availability is applied only after deterministic motion and event time, and is reported separately.

The legacy rule-of-thumb turn-time calculation is not called by this estimator. Every turn is represented by GPS distance and curvature exactly once.

## Reconciliation and invariants

The estimator now rejects a result unless all of the following reconcile within a relative tolerance of 1e-8:

- sum of phase distances equals total timed distance;
- sum of route distances equals total timed distance;
- sum of phase times equals motion plus explicit event time;
- sum of route times equals motion plus explicit event time;
- timed distance equals lifecycle centerline distance;
- continuous-profile time is not less than the instantaneous-speed-change lower bound.

The complete-field result passed with:

- lifecycle distance 3,667,783.146 m;
- timing distance 3,667,783.146 m;
- displayed reconciliation difference -0.000001 m due only to decimal formatting;
- zero geometry gaps;
- zero forced stops;
- zero segments beyond the permitted one-cell exterior allowance;
- 614 flagged manual exterior interventions covering 29.62 mi;
- 180 unique guidance-omitted cells, or 0.805% of the valid field;
- 193 unique cells affected by all omission mechanisms, or 0.863% of the valid field.

## Turn and corner treatment

The final path contains 2,654,336 curved vertices and 1,289,866.8 m of angle-limited distance. It also contains 1,498 short waypoint joins whose raw three-point radius is materially below the tractor's 25 ft radius. These are the previously accepted small operator-correction joins, not full reverse maneuvers.

The timing model caps those joins at maximum steering and 4 mph. It assumes the operator or steering controller rounds each infeasible point locally without reversing, consistent with the decision to omit detailed tractor-scraper kinematics while allowing small in-cab corrections. The raw join and its assumed local rounding remain explicitly reported; the model does not silently assign a faster speed to an impossible radius.

## Acceleration, speed, and availability sensitivity

| Scenario | Driving time | Scheduled time |
|---|---:|---:|
| Baseline | 433.483 h | 541.854 h at 80% availability |
| All operating speeds -10% | 477.650 h | 597.063 h at 80% availability |
| All operating speeds +10% | 397.076 h | 496.345 h at 80% availability |
| Longitudinal acceleration and deceleration halved | 458.748 h | 573.435 h at 80% availability |
| Longitudinal acceleration and deceleration doubled | 410.786 h | 513.483 h at 80% availability |

The instantaneous-speed-change lower bound with all current local phase and angle caps is 369.241 h. Continuous acceleration, braking, and speed transitions add 64.242 h. This is 14.8% of deterministic driving time; it is not being applied once per lifecycle segment. The larger transition share is expected because straight blade-up sections can now approach their 10 or 13 mph payload speed and must physically brake for work or tight geometry.

Operating speed and operational availability dominate uncertainty. Longitudinal acceleration assumptions are secondary but still material.

## Why this cannot equal 205 h

The commercial 205 h result cannot describe this same GPS centerline under these same speed assumptions:

- Driving all 2,279.05 mi at the absolute 13 mph maximum, including cuts, fills, grading, and turns, would still require 175.312 h.
- The current phase and angle caps require at least 369.241 h even with instantaneous acceleration and braking.
- Blade-down work alone requires 71.579 h.
- Explicit lineups, exits, and fill-to-grade turns require another 123.681 h.
- Those work and maneuver phases total 195.260 h, leaving only 9.740 h to drive 1,242.274 transit miles in a 205 h total. That would require more than 127 mph for all transit.
- A 205 h completion would require an 11.117 mph average over the entire 2,279.05 mi path, which is incompatible with 4 mph cutting and turning, 5 mph filling, and 8 mph grading.

Therefore the remaining difference is not a timing-accounting error. It is a difference in route distance, included operations, grading scope, maneuver representation, speed assumptions, or some combination of those. The current guidance geometry is the dominant reason this result is much higher.

## Diesel planning

Do not multiply 541.854 scheduled hours by the full-load fuel rate. Use phase-specific fuel rates when available:

`diesel = sum(phase driving hours * phase fuel rate) + engine-on nondriving hours * idle fuel rate`

If only one blended moving fuel rate is available, use:

`moving diesel = 433.483 h * blended moving gallons per hour`

The 108.371 h availability allowance should be charged at idle rate only for the portion during which the engine is actually running. Refueling, servicing, breaks, weather delays, and off-shift time may have zero tractor fuel burn.

## Validation evidence

- Thirteen analytical and synthetic timing tests cover user-setting mapping, waypoint-density invariance, lifecycle-segment partition invariance, hand-calculated trapezoidal motion, continuous phase transitions, linear turn-angle interpolation, loaded/unloaded full speed, straight lineup speed, curve-sampling stability, manual-assist speed, geometry discontinuity stops, explicit event accounting, and availability separation.
- The complete production guidance test filter passes 51 of 51 tests.
- The independently generated first-ten-route report matches routes 1 through 9 in the full-field route ledger. Route 10 differs only because the limited report must stop there while the full plan carries speed into route 11, confirming that route boundaries do not introduce artificial stops.
- Per-route time is retained for all 3,152 routes. Median drive time is 0.1285 h, p90 is 0.2161 h, p99 is 0.3041 h, and maximum is 0.4148 h.

## Defensible interpretation

The model result is numerically exact for the generated GPS geometry and the stated speed, acceleration, curvature, event, and availability assumptions. It is not a promise that weather, soil response, operator behavior, breakdowns, or measured tractor performance will exactly match those assumptions.

For work-window planning, use 433.483 h as required driving time and select the scheduled window from a measured operational-availability factor. Until site telemetry establishes a better factor, 541.854 h at 80% availability is a transparent planning case rather than hidden time inside the guidance calculation.

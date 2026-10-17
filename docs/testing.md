# Verification plan and evidence

## Automated tests

```bash
python3 -m unittest discover -s tests -v
```

These test the pure Figure-8 math and geofence predicate only. They do not validate PX4 commands or actual flight.

## Dry-run verification

With PX4 SITL, MicroXRCEAgent, and ROS 2 environment active:

```bash
python3 src/search_pattern_offboard.py
```

Expected: local position received, home recorded, and `DRY RUN: telemetry OK; flight commands disabled`.

## Flight acceptance tests (not yet completed)

| Test | Pass criteria | Result |
|---|---|---|
| Preflight | PX4 reports flight readiness without forced arming | PENDING |
| Takeoff | Measured altitude reaches near 5 m | PENDING |
| Search | Recorded XY track follows Figure-8 targets | PENDING |
| Normal landing | Vehicle lands and disarms after mission | PENDING |
| Geofence | A controlled simulated breach is detected | PENDING |
| Failsafe | Landing command accepted and vehicle lands | PENDING |

## Evidence to capture

1. Simulator screenshot showing the X500 in flight.
2. Controller console log with state transitions.
3. PX4 status and command acknowledgments.
4. CSV containing timestamp, target XYZ, actual XYZ, state, and breach flag.
5. Plot of target vs actual XY trajectory, plus tracking error summary.
6. Short demo recording with captions and no unsupported claims.

Do not replace PENDING with PASS until the associated evidence exists.

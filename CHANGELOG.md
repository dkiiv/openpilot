# IQ.Pilot User Changelog

This changelog is written for everyday drivers and focuses on what you will notice on the road.

## February 9, 2026 - IQ.Pilot Launch
- IQ.Pilot v1.0a launched.
- You got first-release IQ.Pilot + Konn3kt integration.
- The UI was rebranded and cleaned up for a more consistent look.
- Unused legacy UI/debug elements were removed.
- Early setup and reliability issues were fixed (better error handling and cleaner setup flow).

## February 10, 2026 - Device Support and Stability
- Better mici and tizi device support.
- You can now use manual QR registration.
- Volkswagen PQ support was enabled and expanded.
- A major bugfix pass improved stability and controls behavior.
- Cruise/button behavior was refined for more consistent response.

## February 11, 2026 - Major Vehicle Expansion
- Tesla control support was expanded.
- Honda MVL tuning (lateral + longitudinal) was added.
- Volkswagen support grew across PQ and MLB with multiple fixes.
- PQ bring-up continued, including Passat NMS-focused improvements.
- More runtime fixes landed (including joystick and icon issues).

## February 12, 2026 - Sensor/Data Fixes
- Steering-angle offset tolerance was increased, helping with temporary steering sensor misalignment.
- Fuel level handling was fixed and improved.
- Additional Volkswagen PQ follow-up fixes were added.

## February 13, 2026 - Driver Monitoring Rollback
- Driver Monitoring was rolled back to previous behavior to reduce over aggressive alerts from a regression caused by a new comma DM model.

## February 15-16, 2026 - Volkswagen PQ Maturity
- Volkswagen PQ received heavy control tuning and bugfixes.
- You should see more consistent engagement and smoother overall behavior.
- Lateral/longitudinal interaction and cruise response were improved.

## February 17, 2026 - Volkswagen Stopping Behavior Tuning
- Volkswagen stopping and braking behavior received additional tuning.
- Comfort and stop-response were further refined for real-world driving.

## February 27, 2026 - Major Refactor
- IQ.Pilot completed a major refactor and is now independent of any other fork.
- Driving logic cleanup removed legacy compatibility paths that could cause inconsistent behavior between vehicles and branches.
- New (way better) Always On Lateral logic!
- Better longitudinal controls for clearer choices for everyday use: `IQ.Pilot`, `IQ.Dynamic`, `IQ.Standard`, and your vehicles `Stock ACC`.
- Device/System controls were reworked for better offroad management, including a timed Force On-Road workflow for diagnostics/testing.
- Volkswagen MEB/MQBEvo platform support!
- Offroad UI and settings UI rework!

## Summary
- IQ.Pilot completed a major refactor and is now independent of any other fork.
- Volkswagen MEB/MQBEvo platform support!
- Offroad UI and settings UI rework!


# Bug-Fix Implementation Plan

Baseline: branch `bug-fixes`, commit `56f6278`, Java 17, WPILib/GradleRIO
2026.2.1, Phoenix 6.26.1.3, PathPlanner 2026.1.2, Choreo 2026.0.3, and
AdvantageKit 26.0.2.

The full finding history remains in [CODE_REVIEW_TRACKER.md](CODE_REVIEW_TRACKER.md).
This document turns its remaining queue into small, reviewable implementation
patches. Each patch should be implemented, externally/native tested where
appropriate, presented with the requested Current/Incoming review block, and
left uncommitted for the user to resolve and commit.

## Current state

- The accepted baseline is commit `56f6278`; the working tree contains the
  uncommitted #5 MT2 innovation-gate patch and the decision-independent #10
  emergency-stop requirement patch for review.
- The intake preset change is accepted in `44b21ae`. `goOut()` and `returnIn()`
  stop the rollers, issue a Motion Magic target once, and release intake
  ownership; this is a modest lifecycle improvement, not a demonstrated match
  failure.
- CTRE-only drivetrain ownership, one-time blue-origin seeding, Limelight
  filtering/publisher refactor, late alliance selection, and bump requirements
  are already accepted.
- Hanger work is explicitly excluded. The hood override finding was withdrawn.
- The resolved #5/#10 patches compile; the focused Limelight fixture has 11
  passing cases and the ownership fixture has 5 passing cases. The durable
  repository currently has no `src/test` suite; high-risk checks remain
  temporary external fixtures.
- Normal auto -> disabled -> teleop already cancels commands that do not run
  while disabled. The optional direct enabled-to-enabled autonomous backstop is
  not a required match fix.
- The valid-shot dashboard status correction is accepted in `f14235b`; D4 range
  clamping versus rejection remains undecided.

## Implementation order

### Phase 0 — Keep the baseline auditable

**Scope:** tracker and build hygiene only.

1. Keep #9 marked fixed at `44b21ae`, #10 core ownership/reset fixed at
   `fe633bf`, #13 status correction fixed at `f14235b`, the deployment/feed
   batch accepted in `800fdcc`/`06d97ea`, and MT1 bootstrap accepted in
   `56f6278`.
2. Run a clean compile/test and conflict-marker scan before every behavioral
   patch.
3. Keep all temporary diagnostic tests outside the repository unless the team
   later requests a durable test suite.

**Exit criteria:** clean working tree before a patch, `git diff --check`, no
source conflict markers, and a successful Gradle verification command.

### Phase 1 — Command ownership and reset safety (#10)

**Current patch:** emergency-stop factories now require the subsystem they
interrupt. D3 persistence/recovery behavior remains intentionally unchanged.

Implement the unambiguous ownership defects first:

1. `outtake()` must require both `intake` and `floor` because it writes both.
2. `zeroPose()` must require `swerve`.
3. `zeroIntake()` must require `intake`.
4. `resetRobot()` must issue direct actuator stop requests, not construct
   `stopFloor()`, `stopFeed()`, and `stopRollers()` commands and discard them.
   Preserve its existing aggregate requirements.
5. Add requirements to the emergency-stop command factories, but settle the
   persistent-stop behavior separately before changing defaults.

**Do not silently decide D3:** a requirement fixes concurrent ownership, but an
instant emergency-stop command is followed by the shooter's default 500 RPM
command on the next scheduler pass. The team must decide whether emergency stop
is momentary or latched and what explicit recovery command clears it.

**Verification:** native scheduler checks for exact requirement sets, interruption
of competing commands, independently latched floor/feeder/roller requests before
and after reset, and emergency-stop behavior under the chosen D3 policy.

### Phase 2 — Make mechanism units explicit (#12)

Do not patch the numeric literals blindly. First preserve the currently intended
active behavior and separate APIs:

1. Replace the overloaded `Floor.Speed` field named `percentOutput` with explicit
   output-fraction/voltage and angular-velocity representations.
2. Fix the active reverse outtake path, currently verified as `-0.5 RPM`, once
   the intended reverse floor behavior is confirmed. The likely desired meaning
   is reverse output fraction, but the physical value must be confirmed.
3. Decide whether the unused `Floor.feedCommand()`/`SubsystemCommands.startFeed()`
   path is intended to command 300 RPM or should use the established 50% voltage
   feed. Never let `300` pass through a voltage overload again.
4. Give `Feeder.Speed` a correctly named API; do not use `percent()` for an RPM
   value.
5. Make sustained motor commands use explicit start/end cleanup. Keep instant
   start/stop helpers only if their latch contract is intentional and documented.

**Verification:** assert Phoenix control request type/value for normal feed,
reverse outtake, startFeed, endFeed, stop, completion, and interruption. Check
that no request above supply voltage is constructed and that active `feed()` still
produces its established 6 V floor / 1980 RPM feeder behavior unless tuning says
otherwise.

### Phase 3 — Vision bootstrap and rejection (#5)

This phase depends on confirming Limelight OS firmware and the team's desired
enabled-bootstrap behavior (D6), but can be developed behind tests:

**Accepted:** the disabled MT1 bootstrap now works independently of MT2 validity
in `56f6278`. The current review patch adds a timestamp-aware translation gate:
CTRE odometry is sampled at the MT2 timestamp and innovations over `1 m` are
rejected. Enabled bootstrap, log-based covariance tuning, and firmware policy
remain separate follow-ups.

1. Read an independent MT1 seed without requiring a valid MT2 tracking frame.
   Keep seed metadata/timestamp attached to the seed and do not fuse it as MT2.
2. Preserve the current disabled one-time blue-origin reset semantics.
3. Add a latency-aware MT2 innovation/outlier policy. Use the estimator pose at
   the measurement timestamp when available, allow normal motion, and reject
   implausible jumps without treating deliberate initialization as tracking.
4. Keep duplicate/out-of-order, timestamp, finite-pose, tag, ambiguity, and
   angular-rate filters.
5. Tune covariance coefficients from real robot logs; do not claim the current
   heuristic coefficients are calibrated.

**Verification:** valid MT1 + invalid MT2 bootstrap, valid MT2 tracking with no
MT1, duplicate timestamps, rapid-turn frames, ordinary delayed motion, and a
large in-bounds jump. Confirm heading behavior with the installed LL4 firmware.

### Phase 4 — Shot-range correctness (#13)

Implement the unambiguous dashboard correction independently:

1. Publish `Target Out of Range = false` when a valid table result is selected.
2. Resolve D4 before changing lookup behavior: retain endpoint clamping or mark
   distances outside 52-215.5 inches invalid.
3. Preserve D1: automated firing feeds as soon as readiness is achieved, or at
   the intentional two-second maximum if readiness is not achieved.

**Verification:** below range, inside range, above range, ready immediately,
never-ready timeout, interruption, and dashboard values.

### Phase 5 — Deployment and build portability (#8, #16, #20)

**Accepted:** #8 chooser filtering/deploy cleanup and #20 nested-navgrid removal
were accepted in `800fdcc`/`06d97ea`. Actual RoboRIO stale-file contents remain
unverified; #16 remains deferred because the tracked JDK path is
machine-specific.

1. Make deploy cleanup remove stale files (`deleteOldFiles = true`) after
   confirming the team's deployment workflow.
2. Make COMP/TEST chooser filtering honor `Constants.autoChooserConfig`, using
   the actual mixed-case auto names or normalize naming consistently. Verify the
   six competition autos and the one test auto.
3. Remove the tracked machine-specific `org.gradle.java.home` from shared
   configuration and provide a local-only setup that does not break this Mac.
4. Remove the duplicate nested navgrid copy; retain the canonical
   `src/main/deploy/pathplanner/navgrid.json`. Confirm deployed cleanup removes
   any old nested copy.

**Verification:** chooser contents in TEST and COMP modes, deploy tree before/
after deletion, a clean build on a second/team JDK setup, and PathPlanner
pathfinding loading the canonical navgrid.

### Phase 6 — Hardware model and odometry-thread performance (#14, #15)

These require evidence rather than guessing values:

1. Confirm installed drive/steer hardware, gear ratio, wheel radius, module
   centers, measured 12 V speed, current limits, and Choreo assumptions.
2. Reconcile `TunerConstants`, `Swerve.PP_CONFIG`, PathPlanner `settings.json`,
   and `ChoreoProject.chor`; regenerate affected assets only after the measured
   source of truth is approved.
3. Rename or correct the misleading `getRobotRelativeSpeeds()` API without
   breaking its field-relative `TargetUtil` caller or AutoBuilder's correctly
   robot-relative `getChassisSpeeds()`.
4. Measure CTRE callback time, robot loop time, and odometry period. Move
   nonessential SmartDashboard/Mechanism2d work out of the odometry callback,
   retaining only cheap snapshot/logging work there.

**Verification:** real robot logs, representative path timing/tracking, callback
timing before/after, no mutable CTRE state shared across threads, and simulation
startup.

### Phase 7 — Lower-risk cleanup and durable tests (#17-#21)

**Accepted:** #21's `endFeed()` status correction was accepted in `800fdcc`.
The other telemetry and state cleanup remains separate.

1. Correct `endFeed()` status flags and dashboard topic capitalization, while
   checking dashboard consumers.
2. Remove redundant Field2d lookup and fake mechanism telemetry only after
   confirming visualization intent.
3. Define the intended axis/reflection semantics before touching unused
   `Landmark.mirrorXAxis/YAxis()` helpers.
4. Remove only genuinely inactive alternatives; keep Choreo assets used by
   PathPlanner `choreoAuto` files.
5. Add durable tests for the high-risk fixes if the team wants them in-repo;
   otherwise preserve the temporary external diagnostic procedure.

## Decisions/evidence still needed

- **D3:** persistent versus momentary emergency stop and recovery behavior.
- **D4:** clamp or reject shot distances outside the calibration table.
- **D5:** authoritative measured drivetrain configuration.
- **D6:** Limelight OS firmware and enabled MT1 bootstrap policy.
- Intended reverse floor output and whether the unused 300-RPM feed path is
  still used.
- Which intake preset bindings/autos the team actually operates; #9 is fixed
  but currently low impact.

## Recommended next patch

First review/resolve the current **#5 MT2 innovation-gate** and **#10
emergency-stop requirement** patches. Then confirm firmware/heading bootstrap
policy, tune covariance from robot logs, and answer D3 before changing
emergency-stop persistence.

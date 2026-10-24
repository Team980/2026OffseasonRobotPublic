# Robot Code Review Tracker

Revalidated **2026-10-08** against accepted code commit **`56f6278`**. The current
working tree contains the uncommitted #5 MT2 innovation-gate patch and the
decision-independent #10 emergency-stop requirement patch for review.

This preserves the numbering and subjects of the original **21-finding audit**
("FRC 2026 Code Review and Fix Priorities"), including corrected conclusions,
completed fixes, remaining evidence, and unanswered design questions. Additional
findings use `A` identifiers rather than renumbering the original audit.

API baseline: Java 17, WPILib/GradleRIO **2026.2.1**, Phoenix 6 **26.1.3**,
PathPlanner **2026.1.2**, Choreo **2026.0.3**, and AdvantageKit **26.0.2**.
Version-specific conclusions below
were checked against installed source jars as well as the documentation listed
under [References](#references).

## Status overview

Original priorities are retained for traceability; the current repair order is
listed separately under [Next fix queue](#next-fix-queue). **Fixed** closes the
reported code defect, not physical-robot validation. **Partial** means an original
finding contains both completed and outstanding work. **Needs decision** records
verified behavior whose desired replacement is not established.

| Original ID | Original priority | Subject | Current status |
| --- | --- | --- | --- |
| 1 | P0 | Duplicate swerve hardware and odometry ownership | Fixed |
| 2 | P0 | Hanger units and startup validation | Excluded |
| 3 | P0 | Alleged unconditional hood override | Withdrawn |
| 4 | P0 | Alliance-safe, correctly oriented, one-time vision seeding | Fixed |
| 5 | P0 | Limelight rejection, metadata, fusion, and heading policy | Partial; MT1 bootstrap fixed, innovation gate pending |
| 6 | P1 | Alliance captured at named-command construction | Fixed |
| 7 | P1 | Empty requirements on deferred bump commands | Fixed |
| 8 | P1 | Stale deployed autos and unused chooser filtering | Fixed in `800fdcc`/`06d97ea`; RoboRIO contents unverified |
| 9 | P1 | Intake position-command completion | Fixed; modest impact |
| 10 | P1 | Missing subsystem ownership | Partial; emergency requirements incoming, D3 persistence open |
| 11 | P2 | Competing simulation IO architectures | Fixed |
| 12 | P2 | Feed/output units and command lifetimes | Partial; floor units fixed, feeder lifetime/API open |
| 13 | P2 | Timed firing fallback and shot-range reporting | Status correction fixed; D4 range policy open |
| 14 | P2 | Conflicting drivetrain models and speed-frame naming | Needs hardware confirmation |
| 15 | P2 | Dashboard/mechanism work on the odometry thread | Open; timing impact unmeasured |
| 16 | P2 | Machine-specific Gradle JDK setting | Open |
| 17 | P2 | Automated regression coverage | Coverage gap |
| 18 | P3 | Incorrect landmark mirror helper | Open; currently unused |
| 19 | P3 | Duplicated and inactive implementations | Partial |
| 20 | P3 | Duplicate nested navgrid | Fixed in `800fdcc` |
| 21 | P3 | Telemetry and unused-state inconsistencies | Partial; endFeed correction fixed in `800fdcc` |

Eight original findings are fixed (#1, #4, #6, #7, #8, #9, #11, #20), five are
partial (#5, #10, #12, #19, #21), #2 is excluded, and #3 is withdrawn. Other
entries retain their individual open, decision-dependent, hardware-dependent, or
coverage status.

## Established decisions and review workflow

- CTRE owns drivetrain hardware, estimation, and simulation. The separate custom
  module/gyro/odometry architecture was removed.
- Field poses remain **blue-origin on both alliances**. Operator perspective is
  separate from field-pose heading.
- Unknown alliance may fall back to Blue for path selection, provided alliance
  is checked again **each command start**. Already running paths keep their goal.
- The Limelight 4 is **landscape mounted**, with robot-space transform configured
  in its web interface. Installed camera firmware remains unconfirmed.
- Hanger repairs and associated tuning are **excluded at the team's request**
  because its physical components malfunction.
- The hood override allegation was a false positive, not an outstanding fix.
- For 20 ms robot-loop work, retain typed NetworkTables publishers created during
  initialization and avoid repeated topic lookup and unnecessary allocation.
- Behavioral patches are reviewed individually using the requested inline
  Current/Incoming conflict-style blocks. The incoming version is validated
  before adding review markers; the user resolves and commits the changes.
- Diagnostic fixtures have been kept outside the repository. Repository test
  files from earlier work were removed by the user; #17 records that distinction.

## Completed patch history

| Commit | Accepted work | Original findings affected |
| --- | --- | --- |
| `3ff2d80` | CTRE-only drivetrain; one-time, blue-origin MT1 pose seed; removal of redundant IO files | #1, #4, #11, part of #19/#21 |
| `96950da` | Removed the commented older LimelightHelpers implementation | Part of #19 |
| `6504032` | Limelight measurement/filtering/uncertainty refactor and cached typed publishers | Part of #5 |
| `1a1322a` | Per-start alliance selection, runtime bump construction, explicit drivetrain requirements, safe AutoBuilder alliance fallback | #6, #7 |
| `44b21ae` | Intake preset commands stop rollers, issue a pivot target once, and release intake ownership | #9 |
| `fe633bf` | Added missing requirements and direct reset actuator stops | Core of #10 |
| `f14235b` | Corrected the valid shot-table dashboard range status | Part of #13 |
| `800fdcc` | Accepted floor unit separation, active reverse output, chooser filtering, end-feed telemetry cleanup, and duplicate navgrid removal | #8, #12, #20, #21 |
| `06d97ea` | Enabled stale deploy-file cleanup | #8 |
| `56f6278` | Independent disabled MT1 bootstrap when MT2 is unavailable | Part of #5 |

`3ff2d80` also committed the machine-specific `gradle.properties` discussed in
#16. Removal of commented hood test code in that commit does not turn #3 into a
previously active defect.

## Accepted intake positioning patch

The team's revised D2 answer authorized the following behavior, which is now
present in `44b21ae`:

- **#9 / revised D2:** `goOut()` and `returnIn()` use `runOnce()` to stop rollers,
  send the pivot target, and release intake ownership on the next scheduler
  pass. Motion Magic completes/holds the movement asynchronously. Roller stop
  occurs during initialization, following the team's latest Incoming edits.
- **#13 / D1:** readiness ends the wait early; otherwise feeding starts at the
  **two-second maximum**. This is confirmed intentional behavior.

The two review blocks were resolved by the user in commit `44b21ae`. The current
source compiles without conflict markers.

The separate `Robot.java` cancellation proposal was withdrawn after verifying
the normal **auto -> disabled -> teleop** sequence. A1 is reclassified as an
optional guard for a direct enabled-to-enabled mode change, not a normal match
transition defect.

**Verification:** the external `LifecycleFixesTest` passed **13 native/scheduler
cases** before the review blocks were resolved:

- The disabled scheduler cycle cancels actual `CompRightBumpToCenter` before
  `teleopInit()` is called, and default drive resumes after teleop enable.
- Active autonomous feed/intake cleanup runs during that disabled cycle.
- Null autonomous leaves independent commands running; already-finished auto
  does not execute cleanup twice.
- Both intake directions finish on the next scheduler pass even while the pivot
  remains off target, release to a default command, preserve the pivot request
  and requested roller stop, and support reuse of the same command.
- Both directions finish when already at target; another command can take intake
  ownership while the pivot still has an outstanding Motion Magic target.
- The existing half-second autonomous deadline retains its duration and group
  requirements even though the position-setting child is instantaneous.
- Actual automated-fire composition feeds early on a supplied ready input and
  after two seconds without readiness; cancellation stops feeding.

The fixture allows `0.1 degree` native position-readback quantization to verify
the pivot is still off target, and invokes AdvantageKit's before/after loop
hooks to advance the deterministic command-timer clock. The early-ready test
supplies the public readiness flag; it verifies sequencing, not physical spin-up.

## Accepted #10 ownership/reset patch

The first implementation patch was intentionally limited to the unambiguous
requirements and reset defects. The incoming source has been validated before
review markers were restored:

- `outtake()` now requires both `intake` and `floor`, matching its actuator writes.
- `zeroPose()` requires `swerve`.
- `zeroIntake()` requires `intake`.
- `resetRobot()` directly sends the same stop requests that its discarded stop
  command factories were intended to send: floor `STOP` voltage, feeder zero
  voltage, and intake roller zero voltage.
- Existing reset aggregate requirements are preserved.
- `emergencyStopIntake()` and `emergencyStopShooter()` are deliberately not part
  of this patch; their ownership and persistent-stop policy remain separate.

**Verification:** before the review blocks were resolved, external
`OwnershipResetFixTest` passed four native/scheduler
cases: exact affected-command requirement sets, outtake interruption of a floor
owner, direct reset of latched floor/feeder/roller/shooter control requests, and
preservation of reset's aggregate ownership. The test used exact WPILib 2026.2.1
command scheduling and Phoenix 6.26.1.3 applied-control types.

The four review blocks were resolved by the user in `fe633bf`. Emergency-stop
ownership/persistence remains an open follow-up under D3; it was not changed by
that commit despite the commit message wording.

## Accepted #13 status patch

This patch was intentionally limited to the inverted dashboard status on the
valid shot-table branch:

- A non-null `Shot` result publishes `Target Out of Range = false`.
- The existing two-second readiness fallback is unchanged.
- No endpoint clamping/rejection behavior is changed; D4 remains open.

**Verification:** external `ShotRangeStatusFixTest` passed for a 100-inch
calibrated distance. During test design, a 0.5 m distance also returned a
non-null endpoint result, confirming that WPILib's `InterpolatingTreeMap` clamps
outside the table and that D4 must remain a separate decision.

The review block was resolved by the user in `f14235b`. No endpoint policy was
changed.

## Original findings, revalidated

### 1. Two independent swerve implementations are controlling the same hardware

**Status: Fixed — `3ff2d80`.**

The original `Swerve` CTRE superclass constructed drivetrain motors, CANcoders,
Pigeon2, odometry thread, and estimator. Separate `ModuleIOTalonFX`,
`GyroIOPigeon2`, and `PhoenixOdometryThread` instances duplicated that ownership.

Current evidence: `RobotContainer` constructs `new Swerve()`; `Swerve` extends
`TunerSwerveDrivetrain`, passes the generated module constants to CTRE, and logs
CTRE state and aggregate odometry validity. The custom `Module`, module/gyro IO
implementations, and separate odometry thread were deleted.

**Verification:** compilation, deleted-identifier searches, native simulation
startup, and subsequent container/scheduler integration tests. CAN/status and
loop-overrun warnings persisted after this change, so the original attribution
of all those warnings to duplicate hardware was not established.

Files: `src/main/java/frc/robot/subsystems/Swerve.java`,
`src/main/java/frc/robot/RobotContainer.java`,
`src/main/java/frc/robot/generated/TunerConstants.java`.

### 2. Hanger position units are incorrect and startup zeroing is unsafe

**Status: Excluded — explicit team instruction.**

The original concern was inch-valued `Hanger.Position` constants (`UP(-25)`,
`HANGING(6)`, `HUNG(0.2)`) being sent as unconverted Phoenix position rotations,
despite an existing `motorAngle()` conversion. Startup encoder validation and
configuration/zeroing order were also questioned.

Retain that original concern for future hardware repair, including encoder
connectivity, position conversion, homing, and verified limits. It is excluded
from the present repair queue and hardware verification; no hanger changes are
approved by this tracker.

File: `src/main/java/frc/robot/subsystems/Hanger.java`.

### 3. Hood output is overwritten to a fixed position every scheduler cycle

**Status: Withdrawn — original audit false positive.**

The purported `leftServo.setPosition(0.6)` and
`rightServo.setPosition(0.6)` overrides were inside commented code. The original
claim that these were active every 20 ms was wrong. Current `Hood.periodic()`
only calls `updateCurrentPosition()`; the old commented test block was removed
in `3ff2d80`.

This closes the specific override allegation. Hood position estimation/tuning
is a separate subject and was not proved incorrect by this finding.

File: `src/main/java/frc/robot/subsystems/Hood.java`.

### 4. Vision-based pose seeding can crash and produces an incorrect red heading

**Status: Fixed for the reported defects — `3ff2d80`, retained in `6504032`.**

The original code unwrapped an unavailable alliance, used `new Rotation2d(180)`
(radians), and repeatedly reset while disabled because `hasSeededPose` was not
used as a guard. Merely changing radians to degrees would not have corrected
the broader field-heading/operator-perspective error.

Current behavior:

- `hasSeededPose` is private, nonstatic, and guards a single reset.
- `Robot.disabledInit()` calls `seedAgain()` once; `disabledPeriodic()` does not
  repeatedly clear the flag.
- The vision command waits for alliance information and a finite, in-bounds MT1
  `seedPose`, then calls `swerve.resetPose(seedPose)` in blue-origin coordinates.
- It returns immediately after reset because that frame's MT2 used the old yaw.
- Camera heading is not passed through an additional `seedFieldCentric()` call
  or red-alliance heading offset.

**Verification:** earlier native Driver Station/camera integration regression
covered heading and seeding behavior. Initial seeding remains **disabled-only**.
Reliable enabled restart/bootstrap and the dependency on MT2 validity are still
tracked under #5; they are not claimed fixed here.

Files: `RobotContainer.java::seedAgain/updateVisionCommand/InBoundsCheck`,
`Robot.java::disabledInit`, `Swerve.java::periodic` under
`src/main/java/frc/robot/` (subsystem files under `subsystems/`).

### 5. Limelight measurement rejection and fusion logic is unreliable

**Status: Partial — refactor accepted in `6504032`; independent MT1 bootstrap
fixed in `56f6278`, innovation gate pending review.**

Original concerns: `Math.min(error1, error1) > 999` was ineffective;
MT1/MT2 both had to see tags; an MT1 pose could be stored with MT2 timestamp and
metadata; heading uncertainty was declared but not applied.

**Completed:** MT2 now supplies its own translation, timestamp, and metadata;
MT1 is consulted for an independent initial heading. Tracking no longer requires
MT1. Finite/bounds/metadata checks, duplicate/out-of-order rejection, frame-age
limits, and rapid-turn rejection were added. Rejected fresh frames are consumed
so a rapid-turn frame cannot be reused after the robot slows down.

The current tunable translation standard deviation is:

```text
distanceFactor = avgTagDist^2 / tagCount
spanFactor     = tagCount > 1 ? 1 + max(0, 1 - tagSpan / 1 m) : 1
motionFactor   = 1 + (abs(yawRate) / 180 deg/s)^2
linearStdDev   = max(0.05 m, 0.08 * distanceFactor * spanFactor * motionFactor)
headingStdDev  = 1.0e7 rad
```

Frames older than `0.25 s`, future by over `0.02 s`, or turning faster than
`360 deg/s` are rejected. Single-tag MT1 seeds require ambiguity at most `0.3`
and camera distance at most `3 m`; multi-tag seeds use the common validity checks.
These covariance coefficients are **initial heuristics requiring robot-log
tuning**, not an experimentally calibrated optimum.

**Outstanding, reproduced:**

1. **No translation innovation gate:** baseline code accepted an in-bounds MT2
   pose `(12, 6) m` from robot pose `(2, 2) m` with two tags at average distance
   `2 m`, despite a jump over `10 m`. The Incoming patch samples CTRE odometry at
   the MT2 timestamp and rejects translation innovations over `1 m`, matching
   Phoenix's stability guidance while leaving missing history available for
   startup. `samplePoseAt()` uses CTRE current-time timestamps; the patch converts
   FPGA timestamps with `Utils.fpgaToCurrentTime()`.
2. **MT1 bootstrap depends on valid MT2:** this was reproduced at baseline: a
    valid MT1 pose `(2, 3, 67 deg)` was blocked by an invalid MT2 `(0, 0, 0)`
    pose because MT2 was checked before MT1. The Incoming patch reads the
    independent seed first and returns it even when MT2 has no usable pose.

**Accepted MT1 bootstrap in `56f6278`:**

- Reads and validates MT1 before MT2 rejection when a disabled pose seed is
  requested.
- Returns a seed-only `Measurement` when MT1 is reliable but MT2 is stale,
  duplicated, or has no valid pose metadata.
- Marks the seed-only result so `RobotContainer` resets odometry but never fuses
  the MT1-only result as an MT2 tracking measurement.
- Keeps the existing rapid-turn rejection, disabled-only seeding, one-time seed
  guard, MT2 tracking filters, and no enabled-bootstrap behavior.

**Incoming innovation-gate patch — review pending:**

- Compares each MT2 translation with the timestamped CTRE odometry pose.
- Rejects translations more than `1 m` away from that pose.
- Accepts ordinary motion within the threshold and accepts valid vision when
  the odometry history does not contain the requested timestamp.
- Compares translation only; MT2 heading remains effectively ignored by the
  existing `1e7 rad` heading standard deviation.

**Heading/IMU constraints:** LL4 mode 1 while disabled seeds its internal IMU to
the external estimator yaw; mode 4 while enabled gently assists internal yaw
with that external reference. Mode 0 is used after an enabled code restart until
disabled seeding has occurred. This does **not** independently establish correct
field heading or repair a wrong estimator heading using MT2 yaw. Enabled one-time
MT1 initialization was discussed but has not been implemented. Firmware must be
confirmed: LLOS **2026.1** fixes mode 1-to-3/4 transitions and invalid MT2 outputs.

**Performance/type decisions:** telemetry and camera-control publishers are
cached; orientation uses a reused six-element array and retains the recommended
flush. `imumode_set` is published as a **double**, matching the project's helper
setter, while local mode telemetry is integer. Avoid assuming a key's prose label
alone establishes its NT wire type. Pose parsing in `LimelightHelpers` still
allocates its result objects/arrays.

**Verification:** the existing vision/publisher regression cases, the accepted
independent-MT1 test, and the new innovation-gate test passed. The new test
covers normal one-meter motion, rejects a larger jump, and accepts valid vision
when timestamped odometry history is unavailable. Enabled bootstrap and
log-based covariance tuning remain outstanding.

Files: `src/main/java/frc/robot/subsystems/Limelight.java`,
`src/main/java/frc/robot/LimelightHelpers.java`,
`src/main/java/frc/robot/RobotContainer.java::updateVisionCommand`.

### 6. Named pathfinding commands capture the alliance too early

**Status: Fixed — `1a1322a`.**

`registerAlliancePathfindCommand()` prebuilds both destinations and registers
`Commands.either(redPath, bluePath, allianceSupplier)`. It selects the current
alliance at initialization rather than startup, while inheriting drivetrain
requirements. AutoBuilder's flip supplier safely uses `Constants.alliance`.

Covered names: `Start`, `Tower`, `Outpost`, `Depot`, `Hub`, `RightBump`, `LeftBump`,
`RightTrench`, `LeftTrench`. Destinations are already alliance-adjusted blue-origin
poses and are not flipped a second time. The shared helper itself is a style
improvement; per-start conditional selection is the correctness fix.

**Verification:** the same preloaded commands were reused with unknown, Red,
Blue, then unknown alliance. Recorded path requests checked translation,
heading, and goal velocity. Active commands retained their goal until canceled
and scheduled again. This was part of an 18-case native/scheduler regression;
physical trajectory following was not measured by the recording pathfinder.

Files: `RobotContainer.java::registerAlliancePathfindCommand`,
`subsystems/Swerve.java` and `Constants.java` under `src/main/java/frc/robot/`.

### 7. Bump commands are deferred with no drivetrain requirement

**Status: Fixed — `1a1322a`.**

Baseline diagnostics printed **empty outer requirements before and after
initialization**, while the generated inner path had `[Swerve]`. A deferred child
is executed through the wrapper's lifecycle, not separately scheduled; its
requirements do not automatically repair an empty outer requirement set.

`SubsystemCommands.toBump()` and `toBump(boolean)` now defer construction with
`Set.of(swerve)`. Controller bindings directly use those wrappers. Position and
alliance are evaluated when they start, including reuse of the nearest-bump
command from a new position. Target geometry, `59 in` offset, `1 m/s` end
velocity, and `(4, 4, 4, 6)` constraints were preserved.

**Verification:** actual scheduler tests proved all three commands interrupt and
exclude default drive, then release it after cancellation. Other cases covered
multiple poses/alliances, explicit side selection, and outside-field completion.

Files: `src/main/java/frc/robot/commands/SubsystemCommands.java::toBump`,
`src/main/java/frc/robot/RobotContainer.java::configureManualDriveBindings`,
`src/main/java/frc/util/BumpTrajectoryGenerator.java`.

### 8. Deployment can leave stale autos on the RoboRIO

**Status: Incoming verified; review pending — configuration defect confirmed; actual
deployed contents remain uninspected.**

The Current side still sets `deleteOldFiles = false` for the deploy file tree and
calls unfiltered `AutoBuilder.buildAutoChooser()`. The Incoming side sets
`deleteOldFiles = true` and uses `buildAutoChooserWithOptionsModifier` to filter
competition options case-insensitively when `autoChooserConfig == COMP`.
`Constants.autoChooserConfig` is `TEST` and has no active effect on chooser
construction; switching it to `COMP` would not currently filter anything.

The source directory contains six competition autos and one test auto. Competition
prefixes mix `COMP` and `Comp`; copying the commented case-sensitive `"COMP"`
filter would hide most of them. PathPlanner documents that the unfiltered chooser
includes old auto files left in the deploy directory.

**Incoming patch/acceptance:** the deployment cleanup and chooser changes are
present in review blocks. The filter uses `Locale.ROOT` and the `comp` prefix, so
the six mixed-case competition autos remain visible while TEST retains all seven
current autos. Native chooser construction verified the TEST option set. Still
verify that an auto deleted from source disappears after deployment and establish
the RoboRIO's actual deployed contents before declaring it already clean.

Files: `build.gradle`, `RobotContainer.java` and `Constants.java` under
`src/main/java/frc/robot/`, `src/main/deploy/pathplanner/autos/`.

### 9. `IntakeOut` and `IntakeIn` manual commands never finish

**Status: Fixed in `44b21ae`; modest lifecycle impact.**

Baseline `Intake.goOut()` and `returnIn()` used `startEnd()` without an end
condition. They required intake and could not naturally finish at the target. Two-controller
POV bindings use `onTrue()`; one-controller shifted POV bindings use
`whileTrue()`. The latter already cancel on release, so the original audit's
blanket description of all bindings as `onTrue()` was too broad.

**Current impact: modest lifecycle cleanup, not a demonstrated match failure.**
The commands have active source bindings, but which controls/autos the team
actually uses has not been established. Trigger-driven `intake()` and manual
left/right POV pivot control are separate commands. Existing `startEnd()` preset
commands are interruptible, so another intake command can still take ownership.

All seven current `.auto` files reference named `IntakeOut`. Their enclosing
deadline groups already bound that command's lifetime: examples include a
**0.5 s wait** in `CompDepot` and bump autos, a **1 s wait** in `CompHubBackShoot`,
and a path deadline in `CompOutpostAndStay`. The earlier blanket description of
these as half-second waits was inaccurate. This change primarily makes preset
commands naturally finish; their original lifetime does not prove the autos
hang. Actual two-controller preset usage should determine its practical priority.

**Current impact: modest lifecycle cleanup, not a demonstrated match failure.**
The commands have active source bindings, but which controls/autos the team
actually uses has not been established. Trigger-driven `intake()` and manual
left/right POV pivot control are separate commands. Existing `startEnd()` preset
commands are interruptible, so another intake command can still take ownership.

All seven current `.auto` files reference named `IntakeOut`. Their enclosing
deadline groups already bound that command's lifetime: examples include a
**0.5 s wait** in `CompDepot` and bump autos, a **1 s wait** in `CompHubBackShoot`,
and a path deadline in `CompOutpostAndStay`. The earlier blanket description of
these as half-second waits was inaccurate. This change primarily makes preset
commands naturally finish; their original lifetime does not prove the autos
hang. Actual two-controller preset usage should determine its practical priority.

**Decision D2 — revised and implemented:** the team requested `runOnce()` because the motor
controller completes the position request asynchronously. This replaces the
earlier wait-until-tolerance proposal. The verified incoming implementations are:

```java
// goOut
return runOnce(() -> {
    set(Speed.STOP.get());
    set(Position.ReadyToIntake);
});

// returnIn
return runOnce(() -> {
    set(Speed.STOP.get());
    set(Position.Start);
});
```

Each command stops rollers and sets the pivot target during initialization,
then finishes on the next scheduler pass. It owns intake briefly while issuing
the requests; it does not retain ownership while waiting for physical movement.
The team's latest Incoming edits moved roller stop into initialization instead
of the former end callback. Motion Magic retains the pivot target.

Native checks covered both directions while far from the goal and already at
the goal, immediate ownership release, default-command resumption, subsequent
command ownership, the requested roller stop, and reuse. Standalone
commands release their requirement on completion; an enclosing command group
retains its aggregate requirements until the group ends. A
`whileTrue()` binding does not restart a naturally finished command while the
button remains held; releasing and pressing it again schedules it again.

Files: `src/main/java/frc/robot/subsystems/Intake.java::goOut/returnIn`,
`src/main/java/frc/robot/RobotContainer.java`, `src/main/deploy/pathplanner/autos/`.

### 10. Several commands access subsystems without requiring them

**Status: Partial — core ownership/reset fixed in `fe633bf`; emergency-stop
requirements incoming, persistence policy remains open.**

| Factory in `SubsystemCommands` | Actual requirements | Unowned write / call site |
| --- | --- | --- |
| `outtake()` | Intake | Also writes floor; active driver/operator bindings |
| `zeroPose()` | Empty | Resets swerve estimator; active driver X binding |
| `zeroIntake()` | Empty | Resets pivot encoder; active driver/operator X bindings |
| `emergencyStopIntake()` | Current empty; Incoming requires intake | Stops pivot; no active binding found |
| `emergencyStopShooter()` | Current empty; Incoming requires shooter | Stops shooter; no active binding found |

Scheduler reproduction: `emergencyStopShooter()` sets `0 V` without canceling
the already running default command; the next loop requests **500 RPM** again.
Adding an instant command's requirements would interrupt the current owner, but
would still allow the default to resume after the instant command finishes.
**Decision D3** is required for a persistent emergency-stop policy.

**Incoming requirement patch — review pending:** both emergency-stop factories
now require the subsystem they interrupt. This fixes concurrent ownership while
deliberately leaving the default-command recovery behavior unchanged until D3 is
answered.

**Related confirmed reset bug:** active `resetRobot()` already requires swerve,
intake, floor, feeder, hood, shooter, and manual-drive coordination. Inside its
action, however, `floor.stopFloor()`, `feeder.stopFeed()`, and
`intake.stopRollers()` merely **construct commands**; they do not execute them.
With independently latched outputs, a reset left floor at `6 V`, feeder at
`1980 RPM`, and intake rollers at `9.6 V`. Cancellation of another command can
mask this by running that command's cleanup.

**Accepted patch:** `outtake`, `zeroPose`, and `zeroIntake` now require
every subsystem they write. Reset directly sends its stop requests and preserves
its aggregate ownership. The Incoming emergency-stop requirement patch is
separate from persistence behavior under D3. Reads of a pose alone do not require
owning swerve; do not apply the ownership rule indiscriminately to read-only
helpers.

File: `src/main/java/frc/robot/commands/SubsystemCommands.java`.

### 11. Simulation uses the wrong IO implementation

**Status: Fixed by architecture removal — `3ff2d80`.**

The old custom `ModuleIOSim` was unused while `ModuleIOTalonFX` existed alongside
CTRE's simulation. Both custom implementations were removed with #1.
`Robot.simulationPeriodic()` updates the **single CTRE drivetrain model** with
`0.02 s` and battery voltage. There is no custom IO stack needing a simulation
branch anymore.

**Verification:** native CTRE container/scheduler tests run successfully with
this architecture. Residual warning investigation and physical/model fidelity
remain distinct from the resolved duplicate simulation ownership.

Files: `src/main/java/frc/robot/Robot.java::simulationPeriodic`,
`src/main/java/frc/robot/subsystems/Swerve.java`.

## Incoming #12 floor-unit patch — review pending

The incoming patch is deliberately limited to the confirmed floor-unit errors:

- Removes the mixed-unit `Floor.Speed.FEEDSPEED(300)` enum value.
- Keeps `Floor.Speed.FEED(0.5)` as an explicit output fraction for the active
  6 V indexing path.
- Adds a dedicated 300 RPM velocity target for `Floor.feedCommand()`.
- Adds `Floor.setPercentOutput(double)` and uses it for active reverse outtake,
  producing −6 V instead of −0.5 RPM.
- Does not change feeder command lifetime or feeder API naming yet.

**Verification:** external `FloorUnitsFixTest` passed three native control-request
cases: dedicated floor feed requests `300 RPM` as `VelocityVoltage`, normal index
requests `6 V`, and active outtake requests `−6 V` and stops on cancellation.

There are review blocks in `Floor.java` for the unit/API split and in
`SubsystemCommands.java` for the active reverse path. The same uncommitted review
batch also contains the #8 deployment/chooser changes, the #20 nested-navgrid
deletion, and the #21 `endFeed()` status correction. Resolve Incoming to apply
the batch. No commit has been made by the agent.

## Incoming #8/#20/#21 deployment and telemetry batch — review pending

The resolved Incoming version was compiled and checked before review markers were
restored:

- `BatchReviewFixTest` passed both native checks: TEST chooser construction
  exposed all seven current autos, and `endFeed()` published both cleanup flags
  as `false`.
- `FloorUnitsFixTest` passed all three existing #12 control-request checks.
- `./gradlew compileJava --console=plain` passed.
- A combined invocation of all temporary fixtures was not used as an acceptance
  command because multiple fixture classes construct the process-global
  PathPlanner `AutoBuilder`; the resulting duplicate-configuration failures are
  test-harness lifecycle interference, not production compilation failures.

The deployment deletion still needs a real RoboRIO/deploy-tree check, and the
COMP chooser branch needs a native test with `autoChooserConfig == COMP`; the
source filter was checked against the actual mixed-case auto names.

### 12. Floor and feeder speed APIs mix percent output and RPM

**Status: Incoming verified; feeder lifetime/API policy remains open.**

| Current operation | Observed control request | Classification |
| --- | --- | --- |
| `Floor.feedCommand()` calls `set(FEEDSPEED(300))` | **3,600 V**, because the enum overload multiplies by 12 | Confirmed unit error; helper reached through currently unused `startFeed()` |
| Active `outtake()` calls `FEED(0.5).rpm().times(-1)` | **-0.5 RPM** | Confirmed inconsistent conversion on active bindings |
| Active `index()` calls `set(FEED(0.5))` | **6 V** | Current intended forward-output path; different from the erroneous helper |
| Active `kick()` calls `Feeder.set(FEED)` | **1980 RPM** | Current intended feeder velocity path |
| `Feeder.Speed.percent()` | Raw RPM numeric value | Misleading API; active stop uses only `STOP(0)` |

The 3,600 V figure is the **requested value**, not physically delivered voltage;
actual motor output is limited by the available supply/controller.

Floor and feeder `feedCommand()` helpers finish instantly while leaving their
motor request latched; feeder retained `1980 RPM` after `end(false)`.
`startFeed()`/`endFeed()` have no active Java caller or named registration in the
current code. In contrast, active `feed()` combines `index()` and `kick()`, whose
`startEnd()` cleanup stops the motors.

**Incoming patch/acceptance:** the floor output fraction and 300 RPM velocity
paths are now distinct. The active reverse path mirrors the established normal
6 V feed as −6 V. Feeder API naming, feeder instant-command latching, and the
unused `startFeed()`/`endFeed()` lifetime policy remain for a later patch.
The failed reset stop calls were fixed in #10.

Files: `src/main/java/frc/robot/subsystems/Floor.java`,
`src/main/java/frc/robot/subsystems/Feeder.java`,
`src/main/java/frc/robot/commands/SubsystemCommands.java`.

### 13. Automated firing feeds even when readiness is not achieved

**Status: Status correction fixed in `f14235b`; D4 range policy remains open.**

`automatedFire()` uses `waitTime(shooter.maxDelay).until(readiness).andThen(feed...)`
with `maxDelay = 2 s`. The wait ends on **readiness OR elapsed timeout**, so an
unready shooter still begins feeding at timeout. Both paths were verified using
the actual composition in native scheduler tests; physical firing performance
has not been measured by these checks.

**Decision D1 — answered:** end the readiness wait early when ready, with a
maximum wait of two seconds. The existing readiness-or-timeout composition
matches the team's intended fallback; feeding at timeout is not an outstanding
bug. `fireNoAim()` also intentionally feeds without aim readiness.

Additional current evidence:

- `PreparePredictShotCommand` now publishes `"Target Out of Range" = false` for
  a non-null shot result. The missing/null branch remains `true`.
- The shot table's sample interval is **52-215.5 inches**. WPILib's
  `InterpolatingTreeMap.get()` returns the nearest endpoint outside that range,
  so `shot != null` does **not** prove the target lies in the calibrated interval.
  Both out-of-range endpoint results and the dashboard status were reproduced.
  **Decision D4:** intentional endpoint clamping or explicit invalid range?
- Phoenix 26.1.3 `ParentDevice` stores the actual control-request reference.
  Native checks confirmed all shooter motors' applied control equals the shared
  velocity request after `setRPM()`, and differs after `eStop()`. The current
  request equality check was **not** proved broken by a supposed cloned request.

**Verified D1:** supplied-ready input feeds early; never-ready input feeds only
after the two-second maximum; cancellation stops floor/feeder requests.
**Remaining acceptance:** settle D4 and check below/inside/above-table distances.
The accepted status correction retains the confirmed timed fallback.

Files: `src/main/java/frc/robot/commands/SubsystemCommands.java::automatedFire`,
`src/main/java/frc/robot/commands/PreparePredictShotCommand.java`,
`src/main/java/frc/robot/subsystems/Shooter.java`.

### 14. PathPlanner has multiple conflicting robot configurations

**Status: Needs hardware confirmation — discrepancies confirmed, correct values unknown.**

| Parameter | CTRE constants / runtime `Swerve.PP_CONFIG` | PathPlanner GUI settings | Choreo project |
| --- | --- | --- | --- |
| Drive ratio | `6.2009569377990434` | `5.143` | `5.143` |
| Wheel radius | `0.0508 m` (2 in) | `0.051 m` | `0.0508 m` |
| Module X magnitude | `0.254 m` (10 in) | `0.26 m` | `0.26035 m` (10.25 in) |
| Module Y magnitude | `0.2286 m` (9 in) | `0.2286 m` | `0.231775 m` (9.125 in) |
| Speed model | `kSpeedAt12Volts = 3 m/s` | `maxDriveSpeed = 5.45 m/s` | `vmax = 4800 RPM` motor speed |
| Runtime/GUI current-model parameter | `SlipCurrent = 40 A` | `driveCurrentLimit = 60 A` | Separate torque model (`tmax = 0.58 N m`) |

The speed entries are not all the same kind of quantity. CTRE's generated
comment explicitly says `kSpeedAt12Volts` is **measured speed**, not desired max
speed. The project also uses it as runtime PathPlanner capability and manual
drive limit. Motor speed, slip current, supply current, and GUI model limits
must not be equated merely because they share similar labels.

`PP_CONFIG` derives radius, ratio, module locations, and slip current from
`TunerConstants`; runtime code does **not** load `settings.json`. All seven
current `.auto` files use `choreoAuto: true`, making the Choreo project and stored
trajectories relevant even though the standalone `AutoRoutines` class is unused.

**Correction to the original constraint inference:** bump constraints allow
`4 m/s`, but PathPlanner 2026.1.2's generated trajectory passes also desaturate
module speeds against `config.moduleConfig.maxDriveVelocityMPS`. That constraint
alone does not prove a generated path requests 4 m/s with a 3 m/s module model.
Imported Choreo paths, however, use their pre-generated states/timing rather
than regenerating them from `PP_CONFIG`; validate/regenerate those assets when
hardware assumptions change.

**Speed-frame correction:** CTRE `SwerveDriveState.Speeds` is robot-centric.
AutoBuilder correctly receives it via `getChassisSpeeds()`.
`getRobotRelativeSpeeds()` calls `fromRobotRelativeSpeeds()` and actually returns
**field-relative** speeds. Its actual caller, `TargetUtil`'s prediction fallback,
needs field-relative velocity. Rename/update the API/caller consistently; simply
returning robot-relative speeds there would introduce a shooting prediction bug.

**Decision D5 / acceptance:** confirm installed module reduction, measured module
centers, effective wheel radius, measured 12 V speed, and relevant current/model
parameters with the team. Reconcile all three models and generated assets from
that evidence, then check representative path timing/tracking on the robot.
The README identifies WCP Swerve X2t but does not resolve every parameter.

Files: `src/main/java/frc/robot/subsystems/Swerve.java`,
`src/main/java/frc/robot/generated/TunerConstants.java`,
`src/main/java/frc/util/TargetUtil.java`,
`src/main/java/frc/util/BumpTrajectoryGenerator.java`,
`src/main/deploy/pathplanner/settings.json`,
`src/main/deploy/choreo/ChoreoProject.chor`.

### 15. Telemetry work is too heavy for the CTRE odometry callback

**Status: Open performance concern — work/thread confirmed; causal timing impact unmeasured.**

Phoenix `registerTelemetry()` invokes its consumer on the odometry thread,
**while holding the state lock**. Its documentation explicitly requires cheap
work. Frequency `0` selects defaults of `250 Hz` on CAN FD or `100 Hz` on CAN 2.0;
actual frequency should be read from telemetry, not assumed.

The current callback calls `SwerveTelemetry.telemeterize()` (nine topic writes
and twelve module-mechanism updates), then performs six additional SmartDashboard
number writes. `SwerveTelemetry` already caches typed publishers and its pose
array; claiming it recreates all publishers every callback would be incorrect.
Repeated field-type publication and the extra SmartDashboard work remain.

**Next change/acceptance:** move dashboard/mechanism publication to a controlled
robot-periodic rate with cached typed publishers, preserving any deliberately
high-rate logging. Measure callback/loop timing and odometry period before and
after. Current warning logs do not establish this callback as their sole cause.

**Threading details:** `getState()` returns CTRE's mutable shared cached state;
`getStateCopy()` copies while locked. Do not hand a retained callback reference
to another thread as if it were an immutable snapshot. The vision consumer
already uses `getStateCopy()`. `Swerve.periodic()` currently uses `getState()`;
review snapshot consistency when changing this code rather than blindly adding
copies at every call site.

`FieldTelemetry` is currently updated from robot-periodic code, not this
registered callback. WPILib `Field2d.getObject()` and `FieldObject2d.setPose()` are
synchronized; a Field2d data race was not established by this audit.

Files: `src/main/java/frc/robot/RobotContainer.java` (registered telemetry),
`src/main/java/frc/util/SwerveTelemetry.java`,
`src/main/java/frc/robot/subsystems/Swerve.java::periodic`,
`src/main/java/frc/robot/FieldTelemetry.java`.

### 16. The committed build is not portable if `gradle.properties` is added

**Status: Open — the previously untracked file is now committed.**

`gradle.properties` contains
`org.gradle.java.home=/Users/ash-iol/wpilib/2026/jdk` and was tracked in `3ff2d80`.
That location works locally but is not a shared team/CI JDK location. The
original conditional "if added" no longer describes its status.

**Next change/acceptance:** move the machine-specific JDK override to local
Gradle/user configuration or use the installed WPILib Java environment, while
keeping the shared project portable. Verify a team machine/CI with its own JDK
installation; a successful build on this Mac does not demonstrate portability.

File: `gradle.properties`. Portable OS detection in `settings.gradle` is already
committed and is a separate change.

### 17. No automated tests cover high-risk code

**Status: Repository coverage gap; external verification exists.**

No `src/test` files currently exist. That is not equivalent to "none of the fixes
were tested": the earlier pose-seed integration regression, nine vision cases,
18 runtime path/scheduler cases, and the diagnostics below were run externally.
The user removed earlier repository test files.

**Follow-up:** if durable regression tests are requested, preserve meaningful
tests of coordinate frames, late alliance, scheduler ownership, motor-request
units, range policy, and independent vision initialization. Hanger tests remain
outside the current scope. This tracker supplies durable evidence/acceptance
criteria but does not create an automatic repository regression suite.

### 18. `Landmark.mirrorXAxis()` contains an incorrect coordinate expression

**Status: Open, inactive helper — low current runtime priority.**

`mirrorXAxis()` returns original **Y as X**, reflected Y, and heading plus 180
degrees. `mirrorYAxis()` also adds 180 degrees. For general reflection headings,
horizontal reflection uses `-theta` and vertical reflection uses `pi - theta`;
adding pi is a 180-degree rotation, not the general reflection rule.

Searches found **no current callers** of either mirror helper. The active
`flipAlliance()` reflects both coordinates and rotates 180 degrees, consistent
with the rotational alliance transformation; #6/#7 checked actual target cases.
PathPlanner also defaults to rotational symmetry. Its rounded `16.54 x 8.07 m`
field dimensions differ from this project's `651.22 x 317.69 in` by less than
1 mm per axis, not a meter-scale red/blue error.

**Next change/acceptance:** define the helper names' intended field-center axes
before fixing/removing them. Check asymmetric X/Y and a non-cardinal heading,
and distinguish mirror operations from alliance rotation.

File: `src/main/java/frc/robot/Landmark.java`.

### 19. Large duplicated and inactive code blocks increase maintenance risk

**Status: Partial — high-impact duplication removed; dormant alternatives remain.**

- The separate drivetrain IO/odometry stack was removed in `3ff2d80`.
- The commented older `LimelightHelpers` implementation was removed in
  `96950da`; the active helper remains necessary.
- `AutoRoutines` still has a valid class/constructor, with routine/configuration
  code commented out; `RobotContainer` does not instantiate it. Calling the
  whole class commented-out Java would be inaccurate.
- Generated `ChoreoTraj` is referenced by the dormant `AutoRoutines` import/code;
  the standalone Choreo factory/follower path has no active construction caller.
- **Choreo `.traj` assets are active through PathPlanner's Choreo autos.** An
  unused generated Java wrapper is not evidence that those assets can be removed.
- Unused imports, commented alternatives, and empty constants remain lower-risk
  cleanup. `Swerve.getAimDirection()` has no current caller.

**Next change/acceptance:** remove only established-unused alternatives after
confirming the intended future architecture, then compile and verify every
active auto's referenced assets still load. Keep cleanup separate from behavior
corrections.

Files: `src/main/java/frc/robot/LimelightHelpers.java`,
`src/main/java/frc/robot/commands/AutoRoutines.java`,
`src/main/java/frc/robot/generated/ChoreoTraj.java`,
`src/main/java/frc/robot/subsystems/Swerve.java`, `src/main/deploy/choreo/`.

### 20. Duplicate deploy file

**Status: Incoming verified; review pending — duplicate and normal loader path
verified.**

Before the Incoming cleanup, these files were byte-for-byte identical:

- `src/main/deploy/pathplanner/navgrid.json`
- `src/main/deploy/pathplanner/deploy/pathplanner/navgrid.json`

SHA-256 for both:
`fd5c3580d0e0af4f29541009bd8114ad682b7a31607e0eab83563ebeeaf01526`.
PathPlanner 2026.1.2 `LocalADStar` loads `pathplanner/navgrid.json` relative to the
deploy directory, not the nested copy.

**Incoming patch/acceptance:** the canonical grid is retained and the confirmed
redundant nested asset is removed in the working tree. Deployment cleanup in #8
must handle any old nested copy already on the RoboRIO (#8). This is not evidence
of two competing runtime pathfinding grids.

### 21. Smaller telemetry and state inconsistencies

**Status: Partial — seed flag integration fixed; other inconsistencies remain.**

| Original subfinding | Current evidence / disposition |
| --- | --- |
| Shooter topic capitalization | `Smartdashboard/Good To Shoot` still uses lowercase `d`, unlike `SmartDashboard`; align intended dashboard consumers when renaming. |
| `endFeed()` cleanup flags | Current still sets `indexing` and `kicking` to **true** after stop; the Incoming batch sets both to **false**. Helper has no current active caller, so impact is lower than active feed ownership/units. |
| Duplicate `FieldTelemetry.getObject("Target")` | Redundant null branch remains. WPILib's getter creates/returns the object; it is not a confirmed null/crash path. Cache the object if simplifying repeated lookup. |
| Fake mechanism poses | `Robot.robotPeriodic()` still allocates/logs zero-pose arrays and a sinusoidal `FinalCompoenentPoses` entry. These are demonstration poses, not sensor-derived mechanism state. Label/remove according to actual visualization intent. |
| Duplicate brownout setting | Same `6.1 V` value is set in REAL setup and again after container construction; redundant, not conflicting values. |
| `hasSeededPose` unused | Resolved by #4; flag actively guards reset and is re-armed once. |
| Other unused/partial state | `hasEnabled`, `isInTeleop`, `lastPose`, and `maxDistanceChange` have no active use; `std/x`, `std/y`, `std/heading` publishers have no writes. Speed-factor fields do have references and should not all be called unused without call-site review. |

**Incoming patch/acceptance:** `endFeed()` now publishes both cleanup flags as
false. Correct any remaining topic values only after checking dashboard
consumers, and distinguish demo logs/unused state from mechanism behavior. Typed
cached publishers remain the preferred hot-loop pattern.

Files: `src/main/java/frc/robot/subsystems/Shooter.java`,
`src/main/java/frc/robot/commands/SubsystemCommands.java::endFeed`,
`src/main/java/frc/robot/FieldTelemetry.java`,
`src/main/java/frc/robot/Robot.java`,
`src/main/java/frc/robot/RobotContainer.java`.

## Additional finding

### A1. Optional explicit autonomous cancellation for direct enabled mode changes

**Status: Reclassified as optional; withdrawn from the match-critical repair queue.**

At baseline, `Robot.autonomousInit()` stored and scheduled `autonomousCommand`,
but `teleopInit()` only updated controller mode and scheduled aim offset; it did
not cancel the stored auto. PathPlanner 2026.1.2 `PathPlannerAuto.isFinished()`
delegates to the inner command and does not end simply because autonomous mode
became teleop.

**Original reproduction:** schedule actual `CompRightBumpToCenter` in simulated enabled
autonomous, retain it in `Robot.autonomousCommand`, switch DS to enabled teleop,
call `robot.teleopInit()`, then run the scheduler. The auto remains scheduled,
still requires swerve, and default drive remains unscheduled. This deliberately
omitted a disabled period and established only the direct enabled-to-enabled
case. Presenting it as a normal match-transition bug was an overstatement.

**Corrected evidence:** on **auto -> disabled -> teleop**, WPILib 2026.2.1
`CommandScheduler.run()` cancels every scheduled command for which
`runsWhenDisabled()` is false during the disabled cycle. `Robot.robotPeriodic()`
already runs the scheduler while disabled. Native tests with the proposed
`teleopInit()` cancellation removed proved the actual auto had ended and released
swerve **before** calling `teleopInit()`. Feed/intake cleanup also ran before
teleop. Commands explicitly allowed to run disabled, including vision, are an
exception to the claim that disabling cancels all commands.

**Decision D7 — corrected after team feedback:** use the existing disabled-cycle
cancellation for the normal match sequence. The extra `Robot.java` proposal and
its review block were removed. WPILib's template explicit cancellation remains
an optional backstop for a direct enabled mode change; no match evidence was
provided that the disabled period is missing, so it is not a required fix here.

File: `src/main/java/frc/robot/Robot.java::autonomousInit/teleopInit`.

## Diagnostic evidence and verification scope

External `AuditEvidenceTest` first passed **11 diagnostic cases** on this
baseline. Two additional scoped cases passed after the remaining API/lifecycle
review. These assertions describe current behavior; a passing defect diagnostic
means the defect was reproduced, **not fixed**. This table records baseline
`1a1322a`; its #9 lifetime assertion is superseded by `LifecycleFixesTest` for
the intake patch. Its A1 assertion covers only a direct enabled-to-enabled switch,
not the disabled match transition, and is not a match-defect acceptance test.

| Case | Durable assertion / finding |
| --- | --- |
| `affectedCommandFactoriesReallyOmitRequirements` | Actual empty/incomplete requirement sets in #10 |
| `floorFeedFactoryActuallyRequests3600Volts` | Floor feed instant command leaves a 3,600 V request (#12) |
| `activeOuttakeActuallyRequestsMinusHalfRpm` | Active reverse floor request is -0.5 RPM; interruption stops it (#12) |
| `resetRobotDoesNotExecuteItsReturnedStopCommands` | Independently latched floor/feeder/rollers remain 6 V / 1980 RPM / 9.6 V (#10) |
| `intakePositionCommandsNeverFinishNaturally` | Both position commands own intake and remain unfinished (#9) |
| `instantaneousFeederCommandLeavesItsVelocityRequestLatched` | Ended feeder instant command still requests 1980 RPM (#12) |
| `emergencyShooterStopIsOverwrittenByTheAlreadyRunningDefault` | Stop does not interrupt default; next loop restores 500 RPM (#10) |
| `emergencyStopsAdvertiseTheSubsystemTheyInterrupt` | Incoming: emergency-stop factories require intake/shooter while D3 persistence remains unchanged (#10) |
| `shotTableClampsBothOutOfRangeEndpointsAndStatusIsAlwaysTrue` | 1 in / 1000 in return table endpoints; valid preshoot reports out of range (#13) |
| `endFeedIncorrectlyPublishesTrueDuringCleanup` | Both stop-status flags true after completion (#21) |
| `aLargeInBoundsMt2JumpIsCurrentlyAccepted` | Over-10 m jump accepted at 0.16 m standard deviation (#5) |
| `aValidMt1SeedIsBlockedByAnInvalidMt2Pose` | Baseline: valid independent seed blocked by invalid MT2 (#5) |
| `returnsReliableMt1SeedWhenMt2HasNoUsablePose` | Incoming: valid MT1 seed returned without an MT2 fusion measurement (#5) |
| `translationInnovationGateAllowsNormalMotionAndRejectsLargeJumps` | Incoming: one-meter translation accepted, larger jump rejected, missing history allowed (#5) |
| `appliedShooterControlRetainsTheActualRequestInstance` | Applied velocity control is the shared request, not a clone (#13 correction) |
| `actualPathPlannerAutoKeepsDrivetrainOwnershipAfterTeleopInit` | Auto excludes default drive only in the tested direct enabled-to-enabled switch; normal disabled transition separately verified (A1 correction) |

Earlier external `LimelightMeasurementTest` covered nine vision/publisher cases;
`RuntimePathTargetsTest` covered 18 alliance/target/requirement/scheduler cases.
Initial seeding also received native DS/camera integration verification before
its repository test was removed. Fixtures used JUnit through GradleRIO's native
test configuration and temporary Gradle init scripts overriding test source
directories, without adding repository tests or changing production behavior.

Local fixture names: `audit-evidence-tests.gradle`,
`limelight-review-tests.gradle`, `path-target-review-tests.gradle`, under the
session's temporary `omnirush` directory. To rerun an available fixture, replace
`/path/to/fixture.gradle` with its actual init-script path:

```sh
./gradlew -I "/path/to/fixture.gradle" test --tests frc.robot.AuditEvidenceTest --console=plain
```

For the intake patch and disabled-transition checks, select `frc.robot.LifecycleFixesTest` instead. The
verified local command was:

```sh
./gradlew -I "/var/folders/kz/knxjpzx90b7cv7t11mp1frhc0000gq/T/omnirush/audit-evidence-tests.gradle" test --tests frc.robot.LifecycleFixesTest --console=plain
```

The temporary sources and Gradle XML reports are not permanent regression
coverage; reports are replaced by later runs. The assertions above preserve
what was checked so the cases can be recreated. Native tests checked control
requests, command lifecycle, NT values, and requested paths. Physical actuator
performance, camera calibration, covariance tuning, and path tracking still
require robot evidence.

## Behavior decisions

| Decision | Needed answer | Relevant IDs |
| --- | --- | --- |
| D1 — answered | End the readiness wait early when ready; maximum wait is 2 s. Retain the current timed fallback. | #13 |
| D2 — revised | Use `runOnce()` to stop rollers and issue the pivot target, then release ownership; let Motion Magic finish asynchronously. Roller stop is in initialization per the team's latest Incoming edits. | #9 |
| D3 | Is an emergency stop persistent until explicit recovery, or a momentary stop followed by default idle? Which controls should expose it? | #10 |
| D4 | Outside 52-215.5 in, should shot lookup clamp endpoints or mark the target invalid? | #13 |
| D5 | What are the installed/measured drivetrain parameters, and which model is authoritative? | #14 |
| D6 | Which LLOS firmware is installed, and should reliable MT1 allow one-time enabled bootstrap when disabled seeding was missed? | #5 |
| D7 — corrected | Normal auto -> disabled -> teleop already cancels ordinary commands during the disabled scheduler cycle. The extra teleop cancellation proposal was withdrawn. | A1 |

D1, D2, and D7 were answered after the completed audit. D3-D6 still require
answers or hardware evidence; the confirmed D1 fallback is intentional behavior.
D2 was subsequently clarified to immediate `runOnce()` completion, superseding
the initially proposed completion-at-position approach.
D7 was subsequently clarified to the ordinary disabled transition, superseding
the proposed explicit teleop cancellation as a required repair.

## Next fix queue

1. **#5:** review the MT2 innovation-gate patch, then confirm firmware/heading
   bootstrap behavior and tune covariance from measured logs.
2. **#10:** settle D3 for emergency-stop persistence and recovery; core ownership
   and reset actuator-stop defects are already fixed.
3. **#12:** repair or retire unused feeder start/stop helpers according to the
   chosen lifetime contract; floor unit separation and active reverse conversion
   are already accepted.
4. **#13:** correct range telemetry and implement the selected shot-range policy
   after D4. The confirmed two-second firing fallback remains intentional.
5. **#16:** move the machine-specific JDK override to local configuration.
6. **#14 / #15:** reconcile measured hardware/model parameters and move/rate-limit
   nonessential odometry-thread telemetry, with timing/path evidence.
7. **#18-#21:** focused inactive-code/deploy/telemetry cleanup; preserve active
   Choreo assets. Decide durable regression coverage (#17) alongside substantial
   behavior fixes.

Hanger (#2) remains excluded and the withdrawn hood claim (#3) remains closed.
For each accepted patch, update the affected IDs with commit, observed behavior,
verification, and remaining conditions instead of replacing this list with a
smaller category-only summary.

## References

- **Exact local sources inspected:** Phoenix `wpiapi-java:26.1.3`
  (`SwerveDrivetrain`, `ParentDevice`, control requests); PathPlanner
  `PathplannerLib-java:2026.1.2` (`AutoBuilder`, `PathPlannerAuto`,
  `PathPlannerTrajectory`, `PathPlannerPath`, `FlippingUtil`, `LocalADStar`);
  WPILib `2026.2.1` (`DeferredCommand`, command compositions,
  `InterpolatingTreeMap`, `Field2d`, `FieldObject2d`); AdvantageKit `26.0.2`
  (`Logger`, `LoggedRobot`, deterministic timestamp/loop hooks). These versioned source jars
  establish exact behavior where moving web API pages differ.
- [WPILib command requirements/lifecycle](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html)
  and [command compositions](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html).
- [WPILib scheduler cancellation during disabled cycles](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html).
- [WPILib project structure and teleop autonomous cancellation](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html).
- [PathPlanner 2026 AutoBuilder, chooser, and stale deploy files](https://pathplanner.dev/pplib-build-an-auto.html),
  [pathfinding](https://pathplanner.dev/pplib-pathfinding.html), and
  [named commands](https://pathplanner.dev/pplib-named-commands.html).
- [Limelight MT2 blue-origin orientation and LL4 IMU modes](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2),
  [NetworkTables API](https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api),
  and [LLOS 2026.1 fixes](https://docs.limelightvision.io/docs/docs-limelight/software-change-log).
- [AdvantageKit vision template](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/Vision.java)
  for distance-squared/tag-count uncertainty scaling; project-specific baseline,
  spread, and motion coefficients remain tunable.

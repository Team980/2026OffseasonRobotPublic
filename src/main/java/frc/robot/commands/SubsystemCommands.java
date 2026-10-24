package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.opencv.core.Rect;
import org.opencv.core.Rect2d;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Landmark;
import frc.robot.RobotContainer;
import frc.robot.Zone;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ManualDriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Feeder.Speed;
import frc.util.BumpTrajectoryGenerator;
import frc.util.DriveInputSmoother;
import frc.util.TargetUtil;
import frc.util.TargetUtil.Target;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final ManualDriveCommand manualDriveCommand;

    private final ManualDriveSubsystem manualDriveSubsystem = new ManualDriveSubsystem();

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/SubsystemStates");
    private final BooleanPublisher aimPublisher = loggingTable.getBooleanTopic("aiming").publish();
    private final BooleanPublisher preshootPublisher = loggingTable.getBooleanTopic("preshooting").publish();
    private final BooleanPublisher indexPublisher = loggingTable.getBooleanTopic("indexing").publish();
    private final BooleanPublisher kickPublisher = loggingTable.getBooleanTopic("kicking").publish();
    private final BooleanPublisher agitatePublisher = loggingTable.getBooleanTopic("agitating").publish();
    private final BooleanPublisher intakeRollerPublisher = loggingTable.getBooleanTopic("intaking (roller)").publish();
    private final BooleanPublisher swerveRequestingPublisher = loggingTable.getBooleanTopic("swerve requesting").publish();

    private static final boolean logData = true;

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        // OneShooter shooter,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        ManualDriveCommand manualDriveCommand
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.manualDriveCommand = manualDriveCommand;
    }

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            null
        );
    }

    // Swerve Commands
    public Command aim() {
        Command aimCommand = new AimAndDriveCommand(swerve);
        aimCommand.addRequirements(manualDriveSubsystem);
        return aimCommand.beforeStarting(() -> {if (logData) aimPublisher.set(true);})
        .finallyDo(() -> {
            if (logData) aimPublisher.set(false);
        });
    }

    public Command aim(Target target) {
        Command aimCommand = new AimAndDriveCommand(swerve, manualDriveCommand, () -> TargetUtil.getTarget(swerve, target));
        aimCommand.addRequirements(manualDriveSubsystem);
        return aimCommand.beforeStarting(() -> {if (logData)aimPublisher.set(true);})
        .finallyDo(() -> {
            if (logData) aimPublisher.set(false);
        });
    }

    public Command faceForward() {
        return Commands.run(
            () -> {
                double x = manualDriveCommand.currentInput.get().left; //driver.getLeftX();
                double y = manualDriveCommand.currentInput.get().forward; //driver.getLeftY();
                if (Math.hypot(x, y) > DriveInputSmoother.translationJoystickDeadband)
                    manualDriveCommand.setLockedHeading(new Rotation2d(-x, y).rotateBy(Rotation2d.k180deg.times(1.5)));
            },
            manualDriveSubsystem
        ).beforeStarting(() -> {swerve.isAiming = true; if (logData) swerveRequestingPublisher.set(true);}).finallyDo(() -> {swerve.isAiming = false; if (logData) swerveRequestingPublisher.set(false);});
    }

    public Command turn45() {
        return new FunctionalCommand(
            () -> {
                // swerve.isAiming = true;
            },
            () -> {
                Rotation2d currentHeading = swerve.getState().Pose.getRotation();
                double nearest45Deg = Math.round((currentHeading.getDegrees() - 45.0) / 90.0) * 90.0 + 45.0;
                manualDriveCommand.setLockedHeading(Rotation2d.fromDegrees(nearest45Deg).rotateBy(swerve.getOperatorForwardDirection()));
            },
            (interrupted) -> {
                // swerve.isAiming = false;
            },
            () -> false,
            manualDriveSubsystem
        ).beforeStarting(() -> {if (logData) swerveRequestingPublisher.set(true);})
        .finallyDo(() -> {if (logData) swerveRequestingPublisher.set(false);});
    }

    public Command zeroPose() {
        return Commands.runOnce(
            () -> swerve.resetPose(new Pose2d(0, 0, Rotation2d.kZero)),
            swerve
        );
    }

    public Command toBump(boolean left) {
        return Commands.defer(() -> BumpTrajectoryGenerator.toBump(swerve, left), Set.of(swerve))
            .beforeStarting(() -> {if (logData) swerveRequestingPublisher.set(true);})
            .finallyDo(() -> {if (logData) swerveRequestingPublisher.set(false);});
    }

    public Command toBump() {
        return Commands.defer(() -> BumpTrajectoryGenerator.toBump(swerve), Set.of(swerve))
            .beforeStarting(() -> {if (logData) swerveRequestingPublisher.set(true);})
            .finallyDo(() -> {if (logData) swerveRequestingPublisher.set(false);});
    }

    public Command slowDrive() {
        return Commands.startEnd(
            () -> {
                manualDriveCommand.setMaxVelocityPercentage(manualDriveCommand.slowDrivePercent);
            },
            () -> {
                if (manualDriveCommand.demoMode.get()) {
                    manualDriveCommand.setMaxVelocityPercentage(0.2);
                } else {
                    manualDriveCommand.setMaxVelocityPercentage(1.0);
                }
            }
        );
    }

    public Command slowRotation() {
        return Commands.startEnd(
            () -> {
                manualDriveCommand.setMaxRotationalRatePercentage(manualDriveCommand.slowRotationPercent);
            },
            () -> {
                if (manualDriveCommand.demoMode.get()) {
                    manualDriveCommand.setMaxRotationalRatePercentage(0.2);
                } else {
                    manualDriveCommand.setMaxRotationalRatePercentage(1.0);
                }
            }
        );
    }

    public Command slowMotion() {
        return Commands.parallel(
            slowDrive(),
            slowRotation()
        );
    }

    public Command xLock() {
        return Commands.run(
            () -> {
                swerve.setControl(
                    new SwerveRequest.SwerveDriveBrake()
                );
            },
            swerve
        ).beforeStarting(() -> {if (logData) swerveRequestingPublisher.set(true);})
        .finallyDo(() -> {if (logData) swerveRequestingPublisher.set(false);});
    }

    public Command snapHeading(Rotation2d angle) {
        return Commands.runOnce(() -> manualDriveCommand.setLockedHeading(angle), manualDriveSubsystem)
            .beforeStarting(() -> {if (logData) swerveRequestingPublisher.set(true);})
            .finallyDo(() -> {if (logData) swerveRequestingPublisher.set(false);});
    }

    // Feed Commands
    public Command index() {
        return Commands.startEnd(
            () -> {
                floor.set(Floor.Speed.FEED);
            },
            () -> {
                floor.set(Floor.Speed.STOP);
            },
            floor 
        ).beforeStarting(() -> {if (logData) indexPublisher.set(true);})
        .finallyDo(() -> {if (logData) indexPublisher.set(false);});
    }

    public Command indexWithPulses() {
        return Commands.sequence(
            index().raceWith(Commands.waitSeconds(1)),
            Commands.waitSeconds(0.125)
        ).repeatedly();
    }

    public Command kick() {
        return Commands.startEnd(
            () -> {
                feeder.set(Feeder.Speed.FEED);
            },
            () -> {
                feeder.set(Feeder.Speed.STOP);
            },
            feeder 
        ).beforeStarting(() -> {if (logData) kickPublisher.set(true);})
        .finallyDo(() -> {if (logData) kickPublisher.set(false);});
    }

    public Command pulseFeed() {
        return Commands.sequence(
            feed().raceWith(Commands.waitSeconds(0.4)),
            Commands.waitSeconds(.5)
        ).repeatedly();
    }

    public Command agitate() {
        return Commands.runOnce(() -> intake.set(Intake.Speed.STOP.get()), intake)
            .andThen(
                Commands.sequence(
                    Commands.runOnce(() -> intake.set(Intake.Position.AGITATE)),
                    Commands.waitUntil(intake::isPositionWithinTolerance),
                    Commands.runOnce(() -> intake.set(Intake.Position.ReadyToIntake)),
                    Commands.waitUntil(intake::isPositionWithinTolerance)
                )
                .repeatedly()
            ).beforeStarting(() -> {
                if (logData) agitatePublisher.set(true);
            })
            .finallyDo(() -> {
                intake.set(Intake.Position.ReadyToIntake);
                intake.set(Intake.Speed.STOP.get());
                if (logData) agitatePublisher.set(false);
            });
    }

    // Intake Commands


    public Command intakeOn() {
        return intake.intakeCommand().beforeStarting(() -> {if (logData) intakeRollerPublisher.set(true);});
    }

    public Command intakeOff() {
        return intake.stopRollers().finallyDo(() -> {if (logData) intakeRollerPublisher.set(false);});
    }

    public Command intake() {
        return Commands.runEnd(
            () -> {
                intake.set(Intake.Speed.INTAKE.get());
                intake.set(Intake.Position.ReadyToIntake);
            },
            () -> intake.set(Intake.Speed.STOP.get()),
            intake
        ).beforeStarting(
            () -> {
                if (logData) intakeRollerPublisher.set(true);
            }
        ).finallyDo(() -> {if (logData) intakeRollerPublisher.set(false);});
    }

    public Command intakeOut() {
        return intake.goOut();
    }

    public Command intakeIn() {
        return intake.returnIn();
    }

    public Command intakeOutManual() {
        return Commands.runEnd(
            () -> intake.setPivotPercentOutput(-0.05), 
            () -> intake.setPivotPercentOutput(0), 
            intake
        );
    }

    public Command intakeInManual() {
        return Commands.runEnd(
            () -> intake.setPivotPercentOutput(0.05), 
            () -> intake.setPivotPercentOutput(0), 
            intake
        );
    }

    public Command outtake() {
        return Commands.runEnd(
            () -> {
                intake.set(-Intake.Speed.INTAKE.get());
                floor.setPercentOutput(-Floor.Speed.FEED.outputFraction());
            },
            () -> {
                intake.set(Intake.Speed.STOP.get());
                floor.set(Floor.Speed.STOP);
            },
            intake,
            floor
        ).beforeStarting(
            () -> {
                if (logData) intakeRollerPublisher.set(true);
            }
        ).finallyDo(() -> {if (logData) intakeRollerPublisher.set(false);});
    }

    public Command zeroIntakeUp() {
        return Commands.runOnce(
            () -> intake.resetDeployEncoder(),
            intake
        );
    }

    public Command zeroIntakeDown() {
        return Commands.runOnce(
            () -> intake.resetDeployEncoderDown(),
            intake
        );
    }

    public Command emergencyStopIntake() {
        return Commands.runOnce(
            () -> intake.stopMotor(),
            intake
        ).finallyDo(() -> {if (logData) intakeRollerPublisher.set(false);});
    }

    public Command resistIntake() {
        return Commands.runEnd(
            () -> intake.set(Intake.Position.ReadyToIntake), 
            () -> intake.setPivotPercentOutput(0), 
            intake
        ).beforeStarting(
            () -> {
                if (logData) intakeRollerPublisher.set(true);
            }
        ).finallyDo(() -> {if (logData) intakeRollerPublisher.set(false);});
    }

    // Preshoot Commands
    public Command preshoot() {
        return new PreparePredictShotCommand(shooter, hood, swerve)
            .beforeStarting(() -> {if (logData) preshootPublisher.set(true);})
            .finallyDo(() -> {if (logData) preshootPublisher.set(false);});
    }

    public Command preshoot(Target target) {
        return new PreparePredictShotCommand(shooter, hood, swerve, ()->target.pos())
            .beforeStarting(() -> {if (logData) preshootPublisher.set(true);})
            .finallyDo(() -> {if (logData) preshootPublisher.set(false);});
    }

    public Command emergencyStopShooter() {
        return Commands.runOnce(
            () -> shooter.setPercentOutput(0),
            shooter
        ).finallyDo(() -> {if (logData) preshootPublisher.set(false);});
    }

    public Command shootManually(double rpm) {
        return shooter.spinUpCommand(rpm)
            .andThen(feed())
            .beforeStarting(() -> {if (logData) preshootPublisher.set(true);})
            .finallyDo(() -> {if (logData) preshootPublisher.set(false);shooter.stop();});
    }

    // Automated Commands / Command Sequences
    public Command preshootAndAim() {
        return Commands.parallel(
            preshoot(),
            aim()
        );
    }

    public Command preshootAndAimHub() {
        return Commands.parallel(
            preshoot(),
            aim(Target.HUB)
        );
    }

    public Command feed() {
        return Commands.parallel(
            kick(),
            index()
        );
    }

    public Command startFeed()
    {
        return Commands.parallel(floor.feedCommand(), feeder.feedCommand())
            .beforeStarting(() -> {
                if (logData) {
                    indexPublisher.set(true);
                    kickPublisher.set(true);
                }
            });
    }

    public Command endFeed()
    {
        return Commands.parallel(floor.stopFloor(), feeder.stopFeed())
            .finallyDo(() -> {
                if (logData) {
                    indexPublisher.set(false);
                    kickPublisher.set(false);
                }
            });
    }

    public Command feedAndAgitate() {
        return Commands.parallel(
            feed(),
            agitate().beforeStarting(Commands.waitSeconds(0.5))
        );
    }

    public Command automatedFire() {
        return Commands.parallel(
            preshootAndAim(),
            Commands.waitTime(shooter.maxDelay)
                .until(
                    () -> shooter.isWithinThresholdAndStableAndAiming(swerve)
                ).andThen(
                    feed()
                )
        );//.finallyDo(() -> intake.set(Intake.Position.Start));
    }

    public Command fireNoAim() {
        return Commands.parallel(
            feed(),
            preshoot()
        );
    }

    public Command aimOffset(DoubleSupplier controllerX, DoubleSupplier controllerY) {
        return Commands.run(
            () -> {
                double x = controllerX.getAsDouble();
                double y = controllerY.getAsDouble();

                double mag = Math.hypot(x, y);
                double rate = Constants.Driving.AIM_OFFSET_SCALE;


                if (mag < Constants.Driving.AIM_OFFSET_DEADBAND) {
                    x = 0;
                    y = 0;
                    mag = 0;
                }
                
                if (mag > 1) {
                    x /= mag;
                    y /= mag;
                    mag = 1;
                }

                double curve = Math.pow(mag, Constants.Driving.AIM_OFFSET_CURVE);
                double magToCurveRatio = mag == 0 ? 0 : curve / mag;
                x *= magToCurveRatio * rate;
                y *= magToCurveRatio * rate;

                TargetUtil.targetOffset = TargetUtil.targetOffset.minus(new Translation2d(y, x).times(rate));
            }
        ).finallyDo(
            () -> {
                TargetUtil.targetOffset = new Translation2d(0, 0);
            }
        );
    }

    public Command resetAimOffset() {
        return Commands.runOnce(
            () -> {
                TargetUtil.targetOffset = new Translation2d(0, 0);
            }
        );
    }

    // Reset Robot
    public Command resetRobot() {
        return Commands.runOnce(
            () -> {
                swerve.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
                intake.stopMotor();
                floor.set(Floor.Speed.STOP);
                feeder.setPercentOutput(Feeder.Speed.STOP.percent());
                hood.setPosition(hood.currentPosition);
                shooter.setPercentOutput(0);
                intake.set(Intake.Speed.STOP.get());
                TargetUtil.targetOffset = new Translation2d(0, 0);
                if (logData) {
                    aimPublisher.set(false);
                    preshootPublisher.set(false);
                    indexPublisher.set(false);
                    kickPublisher.set(false);
                    agitatePublisher.set(false);
                    intakeRollerPublisher.set(false);
                    swerveRequestingPublisher.set(false);
                }
            },
            swerve, intake, floor, feeder, hood, shooter, manualDriveSubsystem
        );
    }
}

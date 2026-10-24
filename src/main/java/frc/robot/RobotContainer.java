// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.traits.HasTalonSignals;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.Idle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoChooserConfig;
import frc.robot.Constants.ControllerSetting;
import frc.robot.Constants.Driving;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PreparePredictShotCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Floor.Speed;
import frc.util.CustomJoystick;
import frc.util.DriveInputSmoother;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static boolean isInTeleop = false;
    public static Pose2d lastPose;
    public static double restrictedSpeed = 0.5;
    public static double unrestrictedSpeed = 1;
    public static double restrictedRotation = 0.6;
    public static double unrestrictedRotation = 1;
    public static double speedFactor = unrestrictedSpeed;
    public static double rotationFactor = unrestrictedRotation;
    public static ControllerSetting controllerSetting;
    public static ManualDriveCommand manualDriveCommand;

    public static boolean driverShiftKey = false;
    public static BooleanPublisher driverShiftKeyLogger = NetworkTableInstance.getDefault().getBooleanTopic("SmartDashboard/ManualConfigs/DriverShiftKey").publish();
    
    // For landmark visualizaton
    //private final PlotLandmarks plotter = new PlotLandmarks();
    public final Swerve swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Floor floor = new Floor();
    public final Feeder feeder = new Feeder();
    public final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    public final Hanger hanger = new Hanger();
    private final SendableChooser<Command> autoChooser;
    public static Command aimOffsetCommand;


    public static final StringPublisher controllerSettingPublisher = NetworkTableInstance.getDefault().getStringTopic("SmartDashboard/ManualConfigs/ControllerSetting").publish(); 
    private final Limelight limelight = new Limelight("limelight-pdp");


    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    public CommandXboxController driver;
    public CommandXboxController operator;

    public static void restrictSpeed() {
        speedFactor = restrictedSpeed;
        DriveInputSmoother.translationJoystickDeadband = DriveInputSmoother.joystickDeadband*speedFactor;
    }
    public static void unrestrictSpeed() {
        speedFactor = unrestrictedSpeed;
        DriveInputSmoother.translationJoystickDeadband = DriveInputSmoother.joystickDeadband*speedFactor;
    }

    public static void restrictRotation()
    {
        rotationFactor = restrictedRotation;
        DriveInputSmoother.rotationJoystickDeadband = DriveInputSmoother.joystickDeadband*rotationFactor;
    }

    public static void unrestrictRotation()
    {
        rotationFactor = unrestrictedRotation;
        DriveInputSmoother.rotationJoystickDeadband = DriveInputSmoother.joystickDeadband*rotationFactor;
    }

    // private final AutoRoutines autoRoutines = new AutoRoutines(
    //     swerve,
    //     intake,
    //     floor,
    //     feeder,
    //     shooter,
    //     hood,
    //     hanger,
    //     limelight
    // );

    public SubsystemCommands subsystemCommands;

    public static PathConstraints namedCommandConstraints = new PathConstraints(0.85, 0.85, 0.85, 0.85);
    public static PathConstraints teleopPath = new PathConstraints(3, 3, 1, 1);
    

    public RobotContainer() {
        if (Constants.ControllerSettingManuallySet == null) {
            if (DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1)) {
                RobotContainer.controllerSetting = ControllerSetting.TWO;
            } else {
                RobotContainer.controllerSetting = ControllerSetting.ONE;
            }
        } else {
            RobotContainer.controllerSetting = Constants.ControllerSettingManuallySet;
        }

        if (RobotContainer.controllerSetting == ControllerSetting.TWO) {
            driverShiftKey = false;
        }

        controllerSettingPublisher.set(RobotContainer.controllerSetting == ControllerSetting.TWO ? "two" : "one");



        if (Robot.isReal()) {
            driver = new CommandXboxController(0);
            operator = new CommandXboxController(1);
        } else {
            driver = new CustomJoystick(CustomJoystick.DEFAULT_DEVICE_OS, 0);
            operator = new CustomJoystick(CustomJoystick.DEFAULT_DEVICE_OS, 1);
        }

        manualDriveCommand = new ManualDriveCommand(
            swerve,
            () -> -driver.getLeftX() * (driverShiftKey ? 0 : 1),
            () -> -driver.getLeftY() * (driverShiftKey ? 0 : 1),
            () -> -driver.getRightX() * (driverShiftKey ? 0 : 1)
        );

        subsystemCommands = new SubsystemCommands(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            manualDriveCommand
        );


        configureBindings();
        registerNamedCommands();
        // autoRoutines.configure();

        // autoChooser = new SendableChooser<>();

        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> Constants.autoChooserConfig == Constants.AutoChooserConfig.COMP
        // ? stream.filter(auto -> auto.getName().startsWith("COMP")) : stream);
        
        
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        //     options -> Constants.autoChooserConfig == Constants.AutoChooserConfig.COMP
        //         ? options.filter(auto -> auto.getName().toLowerCase(Locale.ROOT).startsWith("comp"))
        //         : options
        // );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);
        
        swerve.resetPose(Pose2d.kZero);

        swerve.registerTelemetry(state -> {
            swerveTelemetry.telemeterize(state);
        });
    }

    public void updateSwerveTelemetry() {
        final SwerveDriveState state = swerve.getStateCopy();
        swerveTelemetry.updateMechanismTelemetry(state);

        var flTarget = state.ModuleTargets[0];
        var flActual = state.ModuleStates[0];

        double targetAngle =
            MathUtil.inputModulus(flTarget.angle.getDegrees(), -180, 180);
        double actualAngle =
            MathUtil.inputModulus(flActual.angle.getDegrees(), -180, 180);

        // SmartDashboard.putNumber("FL Target Speed",
        //     flTarget.speedMetersPerSecond);
        // SmartDashboard.putNumber("FL Actual Speed",
        //     flActual.speedMetersPerSecond);

        // SmartDashboard.putNumber("FL Target Angle",
        //     targetAngle);
        // SmartDashboard.putNumber("FL Actual Angle",
        //     actualAngle);

        // SmartDashboard.putNumber("FL Speed Error",
        //     flTarget.speedMetersPerSecond - flActual.speedMetersPerSecond);

        // SmartDashboard.putNumber("FL Angle Error",
        //     targetAngle - actualAngle);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    


    public void registerNamedCommands() {
        registerAlliancePathfindCommand("Start", Landmark.RIGHT_START,
            new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg));
        registerAlliancePathfindCommand("Tower", Landmark.TOWER,
            new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 5 + 36), Inches.of(0), Rotation2d.k180deg));
        registerAlliancePathfindCommand("Outpost", Landmark.OUTPOST,
            new Transform2d(Inches.of(Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 + 1), Inches.of(0), Rotation2d.k180deg));
        registerAlliancePathfindCommand("Depot", Landmark.DEPOT,
            new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.k180deg));
        registerAlliancePathfindCommand("Hub", Landmark.HUB,
            new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero));
        registerAlliancePathfindCommand("RightBump", Landmark.RIGHT_BUMP,
            new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero));
        registerAlliancePathfindCommand("LeftBump", Landmark.LEFT_BUMP,
            new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero));
        registerAlliancePathfindCommand("RightTrench", Landmark.RIGHT_TRENCH,
            new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero));
        registerAlliancePathfindCommand("LeftTrench", Landmark.LEFT_TRENCH,
            new Transform2d(Inches.of(-24 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5 - 2), Inches.of(0), Rotation2d.kZero));
        
        NamedCommands.registerCommand("AimToHub", 
        (subsystemCommands.aim())
        );

        NamedCommands.registerCommand("IntakeOut", 
            (subsystemCommands.intakeOut())
        );
        
        NamedCommands.registerCommand("IntakeIn", 
            subsystemCommands.intakeIn()
        );

        NamedCommands.registerCommand("Intake", 
            (subsystemCommands.intake())
        );
        NamedCommands.registerCommand("PreShoot", 
            (subsystemCommands.preshoot())
        );

        NamedCommands.registerCommand("Shoot", 
            (subsystemCommands.feed())
        );

        NamedCommands.registerCommand("DeployIntake", 
            (Commands.none())
        );
        

        NamedCommands.registerCommand("Agitate", subsystemCommands.agitate());
        
        NamedCommands.registerCommand("AutomatedFire", subsystemCommands.automatedFire());
    }

    /** Prebuild both destinations; select the current alliance each time the command starts. */
    private void registerAlliancePathfindCommand(String name, Landmark landmark, Transform2d adjustment) {
        Command redPath = AutoBuilder.pathfindToPose(
            landmark.get(Alliance.Red, adjustment), namedCommandConstraints, 0);
        Command bluePath = AutoBuilder.pathfindToPose(
            landmark.get(Alliance.Blue, adjustment), namedCommandConstraints, 0);
        // These are already alliance-adjusted blue-origin poses, so do not flip them again.
        // ConditionalCommand inherits both paths' requirements from AutoBuilder.
        NamedCommands.registerCommand(name, Commands.either(
            redPath, bluePath, () -> Constants.alliance.get() == Alliance.Red));
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());
        shooter.setDefaultCommand(shooter.idleShooterCommand());
        swerve.setDefaultCommand(manualDriveCommand);
        // Operator Bindings
        
        // === STICKS ===

        // Left Stick X: 
         aimOffsetCommand = subsystemCommands.aimOffset(
             () -> driverShiftKey ? driver.getLeftX() : operator.getLeftX(),
             () -> driverShiftKey ? driver.getLeftY() : operator.getLeftY()
        );
        // Left Stick Y: 

        // Left Stick Click (L3): 
    



       // Right Stick X: 

        // Right Stick Y: 

        // Right Stick Click (R3): 

        // === BUMPERS & TRIGGERS ===

        // Left Bumper: Preshoot and Aim
        operator.leftBumper()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.preshootAndAimHub());

        // Right Bumper: Fires without aiming
        operator.rightBumper()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.agitate());
        
        // Left Trigger: Agitate
        operator.leftTrigger()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.intake());

        // Right Trigger: Rev Shooter Flywheels
        operator.rightTrigger() // One controller - Parallel quick aim and fire + auto
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.automatedFire());

        // === FACE BUTTONS ===
        
        // Button A: 
        operator.a()
            .onTrue(subsystemCommands.resetAimOffset());
        // Button B: Outtake
        operator.b()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.outtake());
        
        // Button X: Zero Intake
        operator.x()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .onTrue(subsystemCommands.zeroIntakeDown());
       
        // Button Y: 

        // === DPAD (POV) ===

        // DPAD Up: Intake In
        operator.povUp()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .onTrue(subsystemCommands.intakeIn());
        
        // DPAD Down: Intake Out
        operator.povDown()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .onTrue(subsystemCommands.intakeOut());

        // DPAD Left: Intake In Manual 
        operator.povLeft()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.intakeInManual());
        
        // DPAD Right: Intake Out Manual
        operator.povRight()
            .and(()->controllerSetting == ControllerSetting.TWO)
            .whileTrue(subsystemCommands.intakeOutManual());

        // Reset Robot - Back and start
        operator.start()
            .and(operator.back())
            .and(() -> controllerSetting == ControllerSetting.TWO)
            .onTrue(subsystemCommands.resetRobot());

        //Haptic
        Trigger trigger = new Trigger(() -> {
            return RobotController.getBatteryVoltage() <= 8.5;
        });
        trigger.whileTrue(Commands.startEnd(
            () -> {
                double voltMax = 3;
                double voltDiff = 8.5 - RobotController.getBatteryVoltage();
                voltDiff = Math.min(voltMax, voltDiff);
                double percent = voltDiff / 3;
                operator.setRumble(RumbleType.kBothRumble, 0.3 + 0.7 * percent);
            },
            () -> {
                operator.setRumble(RumbleType.kBothRumble, 0);
            }
        ));
    }
    
    /** Allows one vision pose seed while disabled, once a valid measurement is available. */
    public void seedAgain()
    {
        hasSeededPose = false;
    }

    private void configureManualDriveBindings() {
        // === STICKS ===

        // Left Stick X: Drive 
            // Used in manual drive command
            
        // Left Stick Y: (Drive Forward/Back)
            // Used in manual drive command

        // Left Stick Click (L3): 

        // Right Stick X: (Steer/Turn)
            // Used in manual drive command

        // Right Stick Y: 

        // Right Stick Click (R3): 

        // === BUMPERS & TRIGGERS ===


        // Left Bumper: Prepare Cross Bump
        driver.leftBumper() // Driver - Turn 45 degrees
            .and(() -> !driverShiftKey)
            .whileTrue(subsystemCommands.turn45());

        driver.leftBumper() // Operator - Aim and Preshoot
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.preshootAndAimHub());
        
        // Right Bumper: Fuel Mode (faces forward)
        driver.rightBumper() // Driver - Face forward
            .and(() -> !driverShiftKey)
            .whileTrue(subsystemCommands.faceForward());

        driver.rightBumper() // Operator - Fires without aiming
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.agitate());

        // Left Trigger: Precision
        driver.leftTrigger() // Driver - Slow translation
            .and(()-> !driverShiftKey)
            .whileTrue(subsystemCommands.slowDrive());
        
        driver.leftTrigger() // Operator - Intake
            .and(()-> driverShiftKey)
            .toggleOnTrue(subsystemCommands.intake());
       
        // Right Trigger: Slow Swerve
        driver.rightTrigger() // Driver - Slow rotation
            .and(()-> !driverShiftKey)
            .whileTrue(subsystemCommands.slowRotation());

        driver.rightTrigger() // Operator - Automated Fire
            .and(()-> driverShiftKey)
            .whileTrue(subsystemCommands.automatedFire());

        // === FACE BUTTONS ===

        // Button A: (Operator Only) Reset Aim Offset
        driver.a() // Operator - Reset aim offset
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.resetAimOffset());  
        
        // Button B: (Operator Only) Outtake
        driver.b() // Operator - Outtake
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.outtake());

        // Button X: Zero Pose
        driver.x() // Driver - Zero pose
            .and(() -> !driverShiftKey)
            .onTrue(subsystemCommands.zeroPose());

        driver.x() // Operator - Zero Intake
            .and(() -> driverShiftKey)
            .onTrue(subsystemCommands.zeroIntakeDown());
        
        // Button Y: Controller Toggle (One Controller Mode)
        driver.y()
            .and(()->controllerSetting == ControllerSetting.ONE)
            .toggleOnTrue(
                Commands.runEnd(
                    () -> {driverShiftKey = true; driverShiftKeyLogger.set(true);},
                    () -> {driverShiftKey = false; driverShiftKeyLogger.set(false);}
                )
            );

        // === DPAD (POV) ===

        // DPAD Up: intake up / nearest bump
        driver.povUp() // Driver - Nearest bump
            .and(() -> !driverShiftKey)
            .whileTrue(subsystemCommands.toBump());

        driver.povUp()  // Operator - Intake in
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.intakeIn());

        // DPAD Down: 
        driver.povDown() // Operator - Intake out
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.intakeOut());
        
        // DPAD Left: Bump path left
        driver.povLeft() // Driver - Nearest left bump
            .and(() -> !driverShiftKey)
            .whileTrue(subsystemCommands.toBump(true));
        
        driver.povLeft() // Operator - Intake in manual
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.intakeInManual());
        
        // DPAD Right: Bump path right
        driver.povRight() // Driver - Nearest right bump
            .and(() -> !driverShiftKey)
            .whileTrue(subsystemCommands.toBump(false));

        driver.povRight() // Operator - Intake out manual
            .and(() -> driverShiftKey)
            .whileTrue(subsystemCommands.intakeOutManual());
        
        // Start / Menu (right) and Back / View (left)
        driver.start() // Operator - Reset Robot
            .and(() -> driverShiftKey)
            .and(driver.back())
            .and(() -> driverShiftKey)
            .onTrue(subsystemCommands.resetRobot());
    }

    public static boolean hasSeededPose = false;
    final double maxDistanceChange = 0.2;
    public static boolean hasEnabled = false;
    public static double scale = 0.5;
    
    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
           
            measurement.ifPresent(m -> {
                if(DriverStation.isDisabled())
                {
                    //TODO: Check if this also works for the red side
                    Rotation2d rotation = (DriverStation.getAlliance().get() == Alliance.Red) ? new Rotation2d(180) : new Rotation2d(0);
                    Pose2d driverPose = new Pose2d(m.poseEstimate.pose.getTranslation(), rotation);
                    swerve.resetPose(driverPose);
                    swerve.seedFieldCentric(m.poseEstimate.pose.getRotation());
                    hasSeededPose = true;
                }  
                
                if (InBoundsCheck(m.poseEstimate.pose)) {
                    Vector<N3> std = VecBuilder.fill(m.standardDeviations.get(0, 0) * scale, m.standardDeviations.get(1, 0) * scale, m.standardDeviations.get(2, 0) * scale);
                    swerve.addVisionMeasurement(
                        m.poseEstimate.pose,
                        m.poseEstimate.timestampSeconds,
                        std 
                    );
                }
            });
        })
        .ignoringDisable(true);
    }


    public boolean InBoundsCheck(Pose2d visionPose)
    {
        if (visionPose == null
            || !Double.isFinite(visionPose.getX())
            || !Double.isFinite(visionPose.getY())
            || !Double.isFinite(visionPose.getRotation().getRadians())) {
            return false;
        }
        double fieldLength = Constants.FieldConstants.FIELD_X_LENGTH.in(Meter); 
        double fieldWidth = Constants.FieldConstants.FIELD_Y_LENGTH.in(Meter); 
        if (visionPose.getX() < 0 || visionPose.getX() > fieldLength || visionPose.getY() < 0 || visionPose.getY() > fieldWidth) 
        { 
            return false;
        }
        return true;
    }
}

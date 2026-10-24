package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.Utils; 
import com.ctre.phoenix6.swerve.SwerveRequest; 
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants; 
import com.pathplanner.lib.config.RobotConfig; 
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
// added this below
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldTelemetry;
import frc.robot.Landmark;
//import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.util.TargetUtil;

import org.littletonrobotics.junction.Logger;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem 
{
    // private Pose2d[] updatePose2ds = {
    //     getState().Pose,
    //     getState().Pose
    // };
    // private final Time toleranceTime = Seconds.of(0.1);
    // private double timeCheckpoint = Timer.getFPGATimestamp();
    // private double vxMetersPerSecondFieldRelative = 0;
    // private double vyMetersPerSecondFieldRelative = 0;
    // private double omegaRadiansPerSecondFieldRelative = 0;

    //private static double target = 26; used when we were testing limelight targeting 
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    public final Supplier<Pose2d> poseSupplier = () -> this.getState().Pose;

    public boolean isAiming = false;

    

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds pathFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);
    private static double maxAccelaration = 1;
    MotionMagicConfigs magicConfigs = new MotionMagicConfigs();
    private static final double ROBOT_MASS_KG = Units.lbsToKilograms(115);
    private static final double ROBOT_MOI = 6;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG =
    new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1)
                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1),
        getModuleTranslations());

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    private final Alert odometryInvalidAlert =
        new Alert("Swerve odometry is invalid. Check drivetrain device connections.", AlertType.kError);

    /** CTRE owns all drivetrain hardware, pose estimation, and the odometry thread. */
    public Swerve() {
        super(
            TunerConstants.DrivetrainConstants, 
            0,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.1, 0.1, 0.1),
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight
        );
        register();
      //  driveMotor.getConfigurator().apply(config);
        // RobotConfig config;
        //  try{ 
        //     config = RobotConfig.fromGUISettings();
        // } 
        // catch (Exception e) 
        // { 
        //     e.printStackTrace(); 
        //     throw new RuntimeException("Failed to load PathPlanner RobotConfig", e);
        // } 
        AutoBuilder.configure( 
            () -> getState().Pose, //Robot pose supplier 
            pose -> resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose) 
            () -> getChassisSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE 
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards 
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains 
            new PIDConstants(2, 0.0, 0.005), // Translation PID constants 
            new PIDConstants(3, 0.0, 0.005)  /*new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants 
            new PIDConstants(5.0, 0.0, 0.0)*/
            ), // Rotation PID constants
            PP_CONFIG, // The robot configuration 
            // Resolve at command initialization; the existing supplier defaults missing data to Blue.
            () -> Constants.alliance.get() == Alliance.Red,
            this);
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        // Reference to this subsystem to set requirements );

    }
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds; 
    } 

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getState().Speeds,
            getState().Pose.getRotation()
        );
    }

    private void drive(ChassisSpeeds speeds) {
        setControl(
            robotSpeedsRequest
                .withSpeeds(speeds)
        );
    }

//     private final VoltageOut m_voltReq = new VoltageOut(0.0);
//     private final SysIdRoutine m_sysIdRoutine =
//    new SysIdRoutine(
//       new SysIdRoutine.Config(
//          null,        // Use default ramp rate (1 V/s)
//          Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
//          null,        // Use default timeout (10 s)
//                       // Log state with Phoenix SignalLogger class
//          (state) -> SignalLogger.writeString("state", state.toString())
//       ),
//       new SysIdRoutine.Mechanism(
//          (volts) -> getModule(0).getSteerMotor().setControl(m_voltReq.withOutput(volts.in(Volts))),
//          null,
//          this
//       )
//    );
    

//     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//         return m_sysIdRoutine.quasistatic(direction);
//     }

//     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//         return m_sysIdRoutine.dynamic(direction);
//     }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
       // pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            pathFieldSpeedsRequest.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }
    // public static boolean isTargeting() {
    //     // if (currentTargetID == target) {
    //     //     return true;
    //     // }
    //     // return false;
    //     return Math.abs(currentTargetID - target) < 1e-6;
    // }
    // Trying to get the position of field directly from limelight
  //  private final SwerveDrivePoseEstimator m_poseEstimator;
    
    @Override
    public void periodic() 
    {
        //Smartdashboard.putBoolean("IsAiming", TargetUtil.isAiming(this));
        // double currentTime = Timer.getFPGATimestamp();
        // double dt = currentTime - timeCheckpoint;

        // if (dt >= toleranceTime.in(Seconds)){
        //     updatePose2ds[0] = updatePose2ds[1];
        //     updatePose2ds[1] = getState().Pose;
        //     timeCheckpoint = currentTime;
        //     vxMetersPerSecondFieldRelative = (updatePose2ds[1].getX()-updatePose2ds[0].getX())/dt;
        //     vyMetersPerSecondFieldRelative = (updatePose2ds[1].getY()-updatePose2ds[0].getY())/dt;
        //     omegaRadiansPerSecondFieldRelative = (updatePose2ds[0].getRotation().minus(updatePose2ds[0].getRotation())).getRadians()/dt;
        // }

        // Log a snapshot from CTRE's estimator instead of sampling a second hardware stack.
        SwerveDriveState state = getState();
        boolean odometryValid = isOdometryValid();
        Logger.recordOutput("robotPose", state.Pose);
        Logger.recordOutput("Drive/Speeds", state.Speeds);
        Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput("Drive/ModulePositions", state.ModulePositions);
        Logger.recordOutput("Drive/RawHeading", state.RawHeading);
        Logger.recordOutput("Drive/OdometryValid", odometryValid);
        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);
        Logger.recordOutput("Drive/SuccessfulDaqs", state.SuccessfulDaqs);
        Logger.recordOutput("Drive/FailedDaqs", state.FailedDaqs);
        odometryInvalidAlert.set(!odometryValid);
        TargetUtil.logTarget(this);
        //TargetUtil.logGhostBots();
        FieldTelemetry.getInstance().updateRobotPose(state.Pose);
        
        // test tag
        // LimelightHelpers.SetRobotOrientation("limelight", yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        // var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // if (mt2.tagCount > 0) {
        //     this.addVisionMeasurement(mt2.pose. mt2.timestampSeconds);
        // }

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement 
     * 
     *  passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public Rotation2d getAimDirection() {
        Translation2d hubPosition = Landmark.HUB.get().getTranslation();
        Translation2d robotPosition = getState().Pose.getTranslation();
        return hubPosition.minus(robotPosition).getAngle()
                .rotateBy(getOperatorForwardDirection());
    }

    

    
}

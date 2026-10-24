package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.DynamicSlewRateLimiter;
import frc.util.ManualDriveInput;
import frc.util.Stopwatch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Teleop manual drive command for the swerve drivetrain.
 *
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 */
public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING_WITH_MANUAL_ROTATION,
        DRIVING_WITH_LOCKED_HEADING,
        DIVING_WITH_FIXED_TARGET
    }

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final SwerveRequest idleRequest = new SwerveRequest.Idle();
    private final SwerveRequest xLock = new SwerveRequest.SwerveDriveBrake();
    
    public LinearVelocity maxLinearVelocity = Constants.Driving.kMaxSpeed;
    public AngularVelocity maxAngularVelocity = Constants.Driving.kMaxRotationalRate;
    // Normal
    // public LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(4);
    // public AngularAcceleration maxAngularAcceleration = RotationsPerSecondPerSecond.of(2.25);

    // COMP
    public LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(4);//8 //3
    public AngularAcceleration maxAngularAcceleration = RotationsPerSecondPerSecond.of(2.5);//1.5 //3.5

    public double slowDrivePercent = 0.5;
    public double slowRotationPercent = 0.5;
    public double normalDrivePercent = 1.0;
    public double normalRotationPercent = 1.0;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricTargetAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(2, 0, 0.04);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricHeadingCorrection = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(8, 0, 0.3);

    private State currentState = State.IDLING;
    private Optional<Rotation2d> lockedHeading = Optional.empty();
    public final BooleanEntry demoMode = NetworkTableInstance.getDefault().getTable("SmartDashboard/ManualConfigs").getBooleanTopic("Demo Mode").getEntry(false);

    public final Supplier<ChassisSpeeds> desiredVelocities = () -> getDesiredVelocities();
    public final Supplier<ChassisSpeeds> currentVelocities = () -> getCurrentVelocities();
    public final Supplier<ManualDriveInput> currentInput;

    public double prevTime = Timer.getFPGATimestamp();
    private ChassisSpeeds prevRequest = new ChassisSpeeds(0, 0, 0);
    //private ChassisSpeeds prevAccels = new ChassisSpeeds(0, 0,0);

    public ManualDriveCommand(
        Swerve swerve,
        DoubleSupplier leftInput,
        DoubleSupplier forwardInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        this.currentInput = () -> inputSmoother.getSmoothedInput();
        addRequirements(swerve);
        demoMode.set(false);
        
        //SmartDashboard.putData(this);
    }

    public ChassisSpeeds getDesiredVelocities() {
        final ManualDriveInput input = currentInput.get();

        double rawMagnitude = Math.hypot(input.forward, input.left);
        double left = input.left;
        double forward = input.forward;
        double rotation = input.rotation;

        if (rawMagnitude > 1) {
            left /= rawMagnitude;
            forward /= rawMagnitude;
        }
        
        return new ChassisSpeeds(
            forward * maxLinearVelocity.in(MetersPerSecond),
            left * maxLinearVelocity.in(MetersPerSecond),
            rotation * maxAngularVelocity.in(RadiansPerSecond)
        );
    }

    public ChassisSpeeds getCurrentVelocities() {
        return prevRequest;
    }

    public void setCurrentVelocities(ChassisSpeeds speeds) {
        prevRequest = speeds;
    }

    public void setMaxVelocity(LinearVelocity maxVelocity) {
        this.maxLinearVelocity = maxVelocity;
    }

    public void setMaxVelocityPercentage(double percentage) {
        this.maxLinearVelocity = Constants.Driving.kMaxSpeed.times(percentage);
    }
    
    public void setMaxRotationalRatePercentage(double percentage) {
        this.maxAngularVelocity = Constants.Driving.kMaxRotationalRate.times(percentage);
    }

    public void setMaxRotationalRate(AngularVelocity maxRotationalRate) {
        this.maxAngularVelocity = maxRotationalRate;
    }

    public void seedFieldCentric() {
        initialize();
        swerve.seedFieldCentric();
    }

    public void setLockedHeading(Rotation2d heading) {
        lockedHeading = Optional.of(heading);
        currentState = State.DRIVING_WITH_LOCKED_HEADING;
    }

    private void setLockedHeadingToCurrent() {
        final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        setLockedHeading(headingInOperatorPerspective);
    }

    @Override
    public void initialize() {
        currentState = State.IDLING;
        lockedHeading = Optional.empty();
        prevRequest = new ChassisSpeeds(0, 0, 0);
    }

    @Override
    public void execute() {
        final ChassisSpeeds currentRequest = currentVelocities.get();
        final ChassisSpeeds desiredRequest = desiredVelocities.get();

        double xdiff = desiredRequest.vxMetersPerSecond - currentRequest.vxMetersPerSecond;
        double ydiff = desiredRequest.vyMetersPerSecond - currentRequest.vyMetersPerSecond;
        double rdiff = desiredRequest.omegaRadiansPerSecond - currentRequest.omegaRadiansPerSecond;

        double dt = Timer.getFPGATimestamp() - prevTime;
        prevTime = Timer.getFPGATimestamp();



        double maxTranslationDiff = maxLinearAcceleration.in(MetersPerSecondPerSecond) * dt;
        double maxRotationDiff = maxAngularAcceleration.in(RadiansPerSecondPerSecond) * dt;

        double vx = currentRequest.vxMetersPerSecond + MathUtil.clamp(xdiff, -maxTranslationDiff, maxTranslationDiff);
        double vy = currentRequest.vyMetersPerSecond + MathUtil.clamp(ydiff, -maxTranslationDiff, maxTranslationDiff);
        double omega = currentRequest.omegaRadiansPerSecond + MathUtil.clamp(rdiff, -maxRotationDiff, maxRotationDiff);

        ChassisSpeeds newRequest = new ChassisSpeeds(vx, vy, omega);
        setCurrentVelocities(newRequest);

        if (newRequest.omegaRadiansPerSecond != 0) {
            currentState = State.DRIVING_WITH_MANUAL_ROTATION;
            lockedHeading = Optional.empty();
        } else if (newRequest.vxMetersPerSecond != 0 || newRequest.vyMetersPerSecond != 0) {
            currentState = State.DRIVING_WITH_LOCKED_HEADING;
            if (lockedHeading.isEmpty()) setLockedHeadingToCurrent(); // Sets state to DRIVING_WITH_LOCKED_HEADING as well
        } else if (swerve.isAiming) {
            currentState = State.DIVING_WITH_FIXED_TARGET;
        } else {
            currentState = State.IDLING;
            lockedHeading = Optional.empty();
        }

        switch (currentState) {
            case IDLING -> {
                swerve.setControl(xLock);
            }
            case DRIVING_WITH_MANUAL_ROTATION -> {
                swerve.setControl(
                    fieldCentricRequest
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omega)
                );
            }
            case DRIVING_WITH_LOCKED_HEADING -> {
                swerve.setControl(
                    fieldCentricHeadingCorrection
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(lockedHeading.get())
                );
            }
            case DIVING_WITH_FIXED_TARGET -> {
                swerve.setControl(
                    fieldCentricTargetAngle
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(lockedHeading.get())
                );
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Forward Input", () -> inputSmoother.getSmoothedInput().forward, null);
        builder.addDoubleProperty("Left Input", () -> inputSmoother.getSmoothedInput().left, null);
        builder.addDoubleProperty("Rotation Input", () -> inputSmoother.getSmoothedInput().rotation, null);
    }
}

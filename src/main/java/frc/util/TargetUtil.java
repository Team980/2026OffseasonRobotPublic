package frc.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import frc.robot.Constants;
import frc.robot.FieldTelemetry;
import frc.robot.Landmark;
import frc.robot.Zone;
import frc.robot.subsystems.Swerve;

public class TargetUtil {
    private static final Angle kAimTolerance = Degrees.of(5);
    static {
        Logger.recordOutput("targetPose", Target.HUB.pos());
    }
    private static Translation2d prevPos = Target.HUB.pos();
    private static Distance distanceTolerance = Meters.of(0.01); // in meters
    private static Distance cornerYOffset = Inches.of(24);
    private static Distance cornerXOffset = Inches.of(12);
    private static Time futurePredictForTarget = Seconds.of(1);
    public static Translation2d targetOffset = new Translation2d(0, 0);
    private static int periodics = 5; 

    private static PosTimestamp[] prevStamps = new PosTimestamp[periodics];
    private static StructArrayPublisher<Translation2d> ghostBots = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/ghosts", Translation2d.struct).publish();

    // public static SlewRateLimiter slewedMeasurement = new SlewRateLimiter(10);

    public static void initializePrevStamps(Translation2d startLocation) {
        double curTime = Timer.getFPGATimestamp();
        curTime = curTime - curTime % 0.02;
        for (int i = periodics - 1; i > -1; i--) {
            prevStamps[i] = new PosTimestamp(startLocation, curTime - i*0.02);
        }
        logGhostBots();
    }

    public static void shiftPrevStamps() {
        for (int i = 1; i < periodics; i++) {
            prevStamps[i-1] = prevStamps[i];
        }
    }

    public static boolean isAiming(Translation2d target, Swerve swerve) {
        Rotation2d swerveRot = swerve.getState().Pose.getRotation();
        Rotation2d targetRot = target.minus(swerve.getState().Pose.getTranslation()).getAngle();
        double rotDiff = swerveRot.getDegrees() - targetRot.getDegrees();
        if (rotDiff > 180) {
            rotDiff -= 360;
        } else if (rotDiff <= -180) {
            rotDiff += 360;
        }
        return Math.abs(rotDiff) <= kAimTolerance.in(Degrees);
    }

    public static boolean isAiming(Swerve swerve) {
        return isAiming(getTarget(swerve), swerve);
    }

    public static Translation2d getAvgVelocityVector() {
        Translation2d latestPos = prevStamps[periodics-1].pos;
        Translation2d earliestPos = prevStamps[0].pos;
        double timeDiff = prevStamps[periodics-1].timestamp - prevStamps[0].timestamp;

        double avgXVel = (latestPos.getX() - earliestPos.getX()) / timeDiff;
        double avgYVel = (latestPos.getY() - earliestPos.getY()) / timeDiff;

        return new Translation2d(avgXVel, avgYVel);
    }

    public static void logGhostBots() {
        ghostBots.set(Arrays.stream(prevStamps).map(stamp -> stamp.pos).toArray(Translation2d[]::new));
    }

    public static class PosTimestamp {
        public Translation2d pos;
        public double timestamp;

        public PosTimestamp(Translation2d pos, double timestamp) {
            this.pos = pos;
            this.timestamp = timestamp;
        }
    }

    public enum Target {
        LEFT_CORNER(() -> Landmark.LEFT_CORNER.get(new Transform2d(cornerXOffset, cornerYOffset.times(-1), Rotation2d.kZero)).getTranslation()),
        RIGHT_CORNER(() -> Landmark.RIGHT_CORNER.get(new Transform2d(cornerXOffset, cornerYOffset, Rotation2d.kZero)).getTranslation()),
        HUB(() -> Landmark.HUB.get().getTranslation());

        public Supplier<Translation2d> posSupplier;
        Target(Supplier<Translation2d> posSupplier) {
            this.posSupplier = posSupplier;
        }

        public Translation2d pos() {
            return posSupplier.get();
        }
    }

    public static final InterpolatingTreeMap<Distance, Time> distanceToTimeMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            Seconds.of(
                Interpolator.forDouble()
                    .interpolate(startValue.in(Seconds), endValue.in(Seconds), t)
            )
    );

    static
    {
       setNormalStuff(); 
    }
    public static void setNormalStuff() {
        distanceToTimeMap.put(Meters.of(1.5), Seconds.of(0.95));
        distanceToTimeMap.put(Meters.of(2.7), Seconds.of(1.15));
        distanceToTimeMap.put(Meters.of(4.275), Seconds.of(1.38));
        distanceToTimeMap.put(Meters.of(7.5), Seconds.of(1.5));
    }
    public static void setTestStuff() {
        distanceToTimeMap.put(Inches.of(0), Seconds.of(0));
    }

    public static Translation2d getTarget(Swerve swerve, Target target) {
        Translation2d currentTranslation = swerve.getState().Pose.getTranslation();

        if (prevStamps[0] == null) {
            initializePrevStamps(currentTranslation);
        }

        PosTimestamp currentStamp = new PosTimestamp(currentTranslation, Timer.getFPGATimestamp());
        PosTimestamp latestStamp = prevStamps[periodics-1];
        double timeDiff = currentStamp.timestamp - latestStamp.timestamp;
        int numPoints = (int) (timeDiff / 0.02);

        if (numPoints > 0) {
            double dX = currentStamp.pos.getX() - latestStamp.pos.getX();
            double dY = currentStamp.pos.getY() - latestStamp.pos.getY();
            
            double neededPortion = 0.02 / timeDiff;
            double initialX = latestStamp.pos.getX();
            double initialY = latestStamp.pos.getY();
            double periodicDX = dX * neededPortion;;
            double periodicDY = dY * neededPortion;;

            for (int i = 0; i < numPoints; i++) {
                Translation2d newPos = new Translation2d(initialX + periodicDX * (i + 1), initialY + periodicDY * (i + 1));
                shiftPrevStamps();
                prevStamps[periodics - 1] = new PosTimestamp(newPos, latestStamp.timestamp + (i + 1) * 0.02);
            }
        }
        
        Translation2d avgVelocityVector = getAvgVelocityVector();
        double vx = avgVelocityVector.getX();
        double vy = avgVelocityVector.getY();

        Time airTime = distanceToTimeMap.get(
            Meters.of(
                swerve.getState().
                Pose.
                getTranslation().
                getDistance(target.pos())
            )
        );
        Distance xChange = Meters.of(
            vx * airTime.in(Seconds)
        );
        Distance yChange = Meters.of(
            vy * airTime.in(Seconds)
        );

        return target.pos().minus(
            new Translation2d(xChange, yChange)
        ).plus(targetOffset.times(Constants.alliance.get() == Alliance.Blue ? 1 : -1));
    }
    public static Translation2d getTarget(Swerve swerve) {
        Translation2d robotPos = swerve.getState().Pose.getTranslation();
        Translation2d avgVelocityVector;
        try {
            avgVelocityVector = getAvgVelocityVector();
        } catch (NullPointerException e) {
            ChassisSpeeds curentSpeeds = swerve.getRobotRelativeSpeeds();
            avgVelocityVector = new Translation2d(curentSpeeds.vxMetersPerSecond, curentSpeeds.vyMetersPerSecond);
        }
        double vx = avgVelocityVector.getX();
        double vy = avgVelocityVector.getY();
        robotPos = new Translation2d(robotPos.getX()+vx * futurePredictForTarget.in(Seconds), robotPos.getY()+vy * futurePredictForTarget.in(Seconds));
        robotPos = new Translation2d(MathUtil.clamp(robotPos.getX(), 0, Constants.FieldConstants.FIELD_X_LENGTH.in(Meters)), MathUtil.clamp(robotPos.getY(), 0, Constants.FieldConstants.FIELD_Y_LENGTH.in(Meters)));
        Zone respectiveZone = Zone.getRespectiveAllianceZone(swerve.getState().Pose.getTranslation());
        Target target;
        if (respectiveZone.getRect().contains(robotPos)) {
            target = Target.HUB;
        } else {
            target = switch(Zone.getSideHorizontal(robotPos)) {
                case LEFT -> (Constants.alliance.get() == Alliance.Blue ? Target.RIGHT_CORNER : Target.RIGHT_CORNER);
                default -> throw new RuntimeException("somethin wrong in targetutil");
            };
        }
        return getTarget(swerve, target);
    }
    public static void logTarget(Swerve swerve) {
        Translation2d target = getTarget(swerve);
        if (target.getDistance(prevPos) > distanceTolerance.in(Meters)) {
            prevPos = target;
            Logger.recordOutput("targetPose", prevPos);
            FieldTelemetry.getInstance().updateTargetPose(new Pose2d(prevPos, Rotation2d.kZero));
        }
        //Smartdashboard.putNumber("Distance to Target", target.getDistance(swerve.getState().Pose.getTranslation()));
    }
}
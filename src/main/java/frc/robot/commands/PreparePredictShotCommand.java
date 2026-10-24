package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmark;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
//import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.TargetUtil;

public class PreparePredictShotCommand extends Command {
    public static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        // distanceToShotMap.put(Inches.of(52.0), new Shot(2800 + 250, 0.19));
        // distanceToShotMap.put(Inches.of(114.4), new Shot(3275 + 300, 0.40));
        // distanceToShotMap.put(Inches.of(165.5), new Shot(3650 + 350, 0.48));
        // distanceToShotMap.put(Inches.of(215.5), new Shot(4250 + 400, 0.53));
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800 + 250, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275 + 300, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650 + 350, 0.48));
        distanceToShotMap.put(Inches.of(215.5), new Shot(4250 + 400, 0.53));
    }


    private final Shooter shooter;
    private final Hood hood;
    // private final Supplier<Pose2d> robotPoseSupplier;
    // private final Supplier<Translation2d> target;
    private final Swerve swerve;
    // public static double distanceTarget = 0.0;
    // private ChassisSpeeds chassisSpeeds;
    private final Supplier<Translation2d> target;

    //use lambda supplier of () ->
    public PreparePredictShotCommand(Shooter shooter, Hood hood, Swerve swerve, Supplier<Translation2d> target) { //Supplier<Pose2d> robotPoseSupplier, ChassisSpeeds chassisSpeeds, Supplier<Translation2d> target) {
        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.target = target;
        addRequirements(shooter, hood);
        //Smartdashboard.putData(this);
    }

    public PreparePredictShotCommand(Shooter shooter, Hood hood, Swerve swerve) { //Supplier<Pose2d> robotPoseSupplier, ChassisSpeeds chassisSpeeds, Supplier<Translation2d> target) {
        this(shooter, hood, swerve, () -> TargetUtil.getTarget(swerve));
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    private Distance getDistanceToTarget(Supplier<Translation2d> target) {
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Translation2d targetPosition = target.get(); // flipping is handled in targetutil
       // SmartDashboard.putNumber("Distance From Hub", Meters.of(robotPosition.getDistance(hubPosition)).baseUnitMagnitude());
        return Meters.of(robotPosition.getDistance(targetPosition));
    }

    // private Distance getPredictedDistanceToTarget()
    // {
    //     //Time time = distanceToTimeMap.get(getDistanceToTarget(target));

    //     // final Translation2d robotPosition = new Translation2d(robotPoseSupplier.get().getTranslation().getX()
    //     // + time* chassisSpeeds.vxMetersPerSecond,
    //     // robotPoseSupplier.get().getTranslation().getY()
    //     // + time* chassisSpeeds.vyMetersPerSecond);

    //     final Translation2d robotPosition = swerve.getState().Pose.getTranslation();

    //     final Translation2d targetPosition = TargetUtil.getTarget(swerve);

    //     return Meters.of(robotPosition.getDistance(targetPosition));
    // }

     
    // boolean hasHitFirstBatch = false;
  
    @Override
    public void execute() {
        final Distance distanceToTarget = getDistanceToTarget(target);
        final Shot shot = distanceToShotMap.get(distanceToTarget);
        if (shot != null) {
            shooter.setRPM(shot.shooterRPM);
            hood.setPosition(shot.hoodPosition);
        } else {
            hood.setPosition(Hood.kMinPosition);
        }
    }



    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     builder.addDoubleProperty("Distance to Hub (inches)", () -> distanceHub, (distanceHub) -> getDistanceToHub());
    // }

    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stableTimeCounter = -1;
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}

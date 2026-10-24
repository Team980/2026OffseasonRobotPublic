package frc.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Landmark;
import frc.robot.Zone;
import frc.robot.subsystems.Swerve;

public class BumpTrajectoryGenerator {
    public static final PathConstraints bumpPathConstraints = new PathConstraints(4, 4, 4, 6);// 4, 6, 8, 12
    public static final double bumpOffset = Inches.of(55).in(Meters);
    public static final double endVelocity = 1;//3.5; // m/s
    public static final StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic("Target Bump", Pose2d.struct).publish();
    public static final BooleanArrayPublisher autoConditions = NetworkTableInstance.getDefault().getTable("SmartDashboard").getBooleanArrayTopic("home, left, front").publish();
    public static final double fieldXLength = Constants.FieldConstants.FIELD_X_LENGTH.in(Meters), fieldYLength = Constants.FieldConstants.FIELD_Y_LENGTH.in(Meters);

    public static Command toBump(Swerve swerve, boolean left) {
        if (!Zone.FIELD.getRect().contains(swerve.getState().Pose.getTranslation())) {
            return Commands.none();
        }
        if (Constants.alliance.get() == Alliance.Red) {
            left = !left;
        }
        Translation2d pos = swerve.poseSupplier.get().getTranslation();
        boolean home = Zone.BOTTOM.getRect().contains(pos);
        boolean front = !(Zone.NEUTRAL.getRect().contains(pos));

        Pose2d newPose = new Pose2d(
            (Landmark.LEFT_BUMP.get(Alliance.Blue).getX() - (front ? bumpOffset : -bumpOffset)) * (home ? 1 : -1) + (home ? 0 : fieldXLength),
            Landmark.LEFT_BUMP.get(Alliance.Blue).getY() * (left ? 1 : -1) + (left ? 0 : fieldYLength),
            (Landmark.LEFT_BUMP.get(Alliance.Blue).getRotation().rotateBy(front ? Rotation2d.kZero : Rotation2d.k180deg).rotateBy(home ? Rotation2d.kZero : Rotation2d.k180deg).rotateBy(Rotation2d.k180deg)).rotateBy(Rotation2d.kZero)
        );
        posePub.set(newPose);
        autoConditions.set(new boolean[] {home, left, front});
        return AutoBuilder.pathfindToPose(
            newPose,
            bumpPathConstraints,
            endVelocity
        );
    }

    public static Command toBump(Swerve swerve) {
        boolean res = Zone.LEFT.getRect().contains(swerve.getState().Pose.getTranslation());
        return toBump(swerve, Constants.alliance.get() == Alliance.Blue ? !res : res);
    }
}

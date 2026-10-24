package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import org.ejml.dense.row.linsol.AdjustableLinearSolver_DDRM;

public enum Landmark {
    // Note: Bumper width is 35 inches
    // In the 2026 game, the origin is the bottom right corner from the blue alli ance's perspective.
    OUTPOST(new Pose2d(Inches.of(0.32), Inches.of(26.22), Rotation2d.fromDegrees(0))),
    DEPOT(new Pose2d(Inches.of(0.32), Inches.of(234.32), Rotation2d.fromDegrees(0))),
    TOWER(new Pose2d(Inches.of(0.32), Inches.of(147.47), Rotation2d.fromDegrees(0))),
    HUB(new Pose2d(Inches.of(182.105), Inches.of(158.845), Rotation2d.fromDegrees(0))),
    RIGHT_BUMP(new Pose2d(Inches.of(182.105), Inches.of(98.845), Rotation2d.fromDegrees(0))),
    LEFT_BUMP(new Pose2d(Inches.of(182.105), Inches.of(218.845), Rotation2d.fromDegrees(0))),
    RIGHT_TRENCH(new Pose2d(Inches.of(182.105), Inches.of(25.375), Rotation2d.fromDegrees(0))),
    LEFT_TRENCH(new Pose2d(Inches.of(182.105), Inches.of(292.815), Rotation2d.fromDegrees(0))),

    // The robot's bumper is expected to overlap since the line's thickness will extend it a little towards the alliance side
    RIGHT_START(new Pose2d(Inches.of(158.34 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5), Inches.of(25.375), Rotation2d.fromDegrees(0))),
    LEFT_START(new Pose2d(Inches.of(158.34 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5), Inches.of(292.815), Rotation2d.fromDegrees(0))),
    MIDDLE_START(new Pose2d(Inches.of(158.34 - Constants.RobotDimensions.BUMPER_WIDTH.in(Inches)*0.5), Inches.of(158.845), Rotation2d.fromDegrees(0))),

    LEFT_CORNER(new Pose2d(Inches.of(0), Constants.FieldConstants.FIELD_Y_LENGTH, Rotation2d.fromDegrees(0))),
    RIGHT_CORNER(new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(0)));

    private Pose2d bluePose;
    private Pose2d redPose;

    Landmark(Pose2d bluePose) {
        this.bluePose = bluePose;
    }

        
    public static Pose2d flipAlliance(Pose2d original) {
        return new Pose2d(
            Constants.FieldConstants.FIELD_X_LENGTH.minus(original.getMeasureX()), // flip X only - also flip y?
            Constants.FieldConstants.FIELD_Y_LENGTH.minus(original.getMeasureY()), 
            original.getRotation().plus(Rotation2d.fromDegrees(180)) // rotate heading
        );
    }

    public static Translation2d flipAlliance(Translation2d original) {
        return new Translation2d(
            Constants.FieldConstants.FIELD_X_LENGTH.minus(original.getMeasureX()),
            Constants.FieldConstants.FIELD_Y_LENGTH.minus(original.getMeasureY())
        );
    }

    public static Pose2d mirrorYAxis (Pose2d original) {
        return new Pose2d(
            Constants.FieldConstants.FIELD_X_LENGTH.minus(original.getMeasureX()),
            original.getMeasureY(), 
            original.getRotation().plus(Rotation2d.fromDegrees(180)) 
        );
    }

    public static Pose2d mirrorXAxis (Pose2d original) {
        return new Pose2d(
            original.getMeasureY(),
            Constants.FieldConstants.FIELD_Y_LENGTH.minus(original.getMeasureY()), 
            original.getRotation().plus(Rotation2d.fromDegrees(180)) 
        );
    }

    public Pose2d get() {
        final Alliance alliance = Constants.alliance.get();

        if (alliance==Alliance.Red) {
            if (redPose == null)
                redPose = flipAlliance(bluePose);
            return redPose;
        } 
        return bluePose;
    } 
    
    public Pose2d get(Alliance alliance) {
        if (alliance == Alliance.Red) {
            if (redPose == null)
                redPose = flipAlliance(bluePose);
            return redPose;
        }
        return bluePose;
    }
    public Pose2d get(Transform2d adjustment) {
        return get().plus(adjustment);
    }
    public Pose2d get(Alliance alliance, Transform2d adjustment) {
        return get(alliance).plus(adjustment);
    }
}

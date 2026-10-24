package frc.robot;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Inches;

public enum Zone {
    FIELD(new Rectangle2d(
        new Translation2d(0, 0),
        new Translation2d(Constants.FieldConstants.FIELD_X_LENGTH, Constants.FieldConstants.FIELD_Y_LENGTH))
    ),
    LEFT(new Rectangle2d(
        new Translation2d(0, 0), 
        new Translation2d(Constants.FieldConstants.FIELD_X_LENGTH, Constants.FieldConstants.FIELD_Y_LENGTH.times(0.5)))
    ),
    RIGHT(new Rectangle2d(
        new Translation2d(Inches.of(0), Constants.FieldConstants.FIELD_Y_LENGTH), 
        new Translation2d(Constants.FieldConstants.FIELD_X_LENGTH, Constants.FieldConstants.FIELD_Y_LENGTH.times(0.5)))
    ),
    TOP(new Rectangle2d(
        new Translation2d(Constants.FieldConstants.FIELD_X_LENGTH, Inches.of(0)), 
        new Translation2d(Constants.FieldConstants.FIELD_X_LENGTH.times(0.5), Constants.FieldConstants.FIELD_Y_LENGTH))
    ),
    BOTTOM(new Rectangle2d(
        new Translation2d(Constants.FieldConstants.FIELD_X_LENGTH.times(0.5), Inches.of(0)), 
        new Translation2d(Inches.of(0), Constants.FieldConstants.FIELD_Y_LENGTH))
    ),
    BLUE(new Rectangle2d(
        new Translation2d(Inches.of(0), Inches.of(0)), 
        new Translation2d(Inches.of(156.61), Constants.FieldConstants.FIELD_Y_LENGTH))
    ),
    RED(new Rectangle2d(
        Landmark.flipAlliance(new Translation2d(Inches.of(0), Inches.of(0))), 
        Landmark.flipAlliance(new Translation2d(Inches.of(156.61), Constants.FieldConstants.FIELD_Y_LENGTH)))
    ),
    NEUTRAL(new Rectangle2d( // the actual neutral zone is actually starting from the middle of the bump, but in order to not leave a gap, we're going to start it from the alliance zones
        new Translation2d(Inches.of(156.61), Constants.FieldConstants.FIELD_Y_LENGTH), 
        Landmark.flipAlliance(new Translation2d(Inches.of(156.61), Constants.FieldConstants.FIELD_Y_LENGTH))
    ));

    private Rectangle2d rect;

    Zone(Rectangle2d rect) {
        this.rect = rect;
    }

    public static Zone getSideHorizontal(Translation2d robotPose) {
        if (LEFT.rect.contains(robotPose)) {
            return LEFT;
        } else {
            return RIGHT;
        }
    }

    public static Zone getSideVertical(Translation2d robotPose) {
        if (TOP.rect.contains(robotPose)) {
            return TOP;
        } else {
            return BOTTOM;
        }
    }

    public Zone getAllianceRelative() {
        Alliance alliance = Constants.alliance.get();
        if (alliance == Alliance.Red) {
            if (this == TOP) {
                return BOTTOM;
            } else if (this == BOTTOM) {
                return TOP;
            } else if (this == LEFT) {
                return RIGHT;
            } else if (this == RIGHT) {
                return LEFT;
            }
            throw new IllegalStateException("Cannot get alliance relative for zone: " + this);
        }
        return this;
    }

    public static Zone getAllianceZone(Translation2d robotPose) {
        if (BLUE.rect.contains(robotPose)) {
            return BLUE;
        } else if (RED.rect.contains(robotPose)) {
            return RED;
        } else {
            return NEUTRAL;
        }
    }

    public static Zone getRespectiveAllianceZone(Translation2d robotPose) {
        return Constants.alliance.get() == Alliance.Red ? Zone.RED : Zone.BLUE;
    }

    public Rectangle2d getRect() {
        return this.rect;
    }
}

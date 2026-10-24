// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import javax.sql.rowset.serial.SerialBlob;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Inches;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static Supplier<Alliance> alliance = () -> DriverStation.getAlliance().orElse(Alliance.Blue);

    public static enum ControllerSetting {
        // 2 Controllers: 1 driver, 1 operator
        TWO,
        // 1 Controller: With a shortcut button
        ONE
    }
    public static final ControllerSetting ControllerSettingManuallySet = null; // Set to null for autodetect

    public static enum AutoChooserConfig {
        // In competition: filter necessary
        COMP,
        // In testing: keep all
        TEST
    }

    public static final AutoChooserConfig autoChooserConfig = AutoChooserConfig.TEST; // Set to TEST for testing, COMP for competition

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
    public static class Motors {
    }
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1.5);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);

        public static final double AIM_OFFSET_DEADBAND = 0.15;
        public static final double AIM_OFFSET_CURVE = 2;
        public static final double AIM_OFFSET_SCALE = 0.3;
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class KrakenX44
    {
        public static final AngularVelocity kFreeSpeed = RPM.of(7500);
    }

    public static class FieldConstants {
        public static final Distance FIELD_X_LENGTH = Inches.of(651.22);
        public static final Distance FIELD_Y_LENGTH = Inches.of(317.69);
    }

    public static class RobotDimensions {
        public static final Distance BUMPER_WIDTH = Inches.of(35); // Bumpers add 3.5 each side so 25.5 + 7 = 32.5, but maybe add a little extra for the bumper mounts and such
        public static final Distance ROBOT_WIDTH = Inches.of(25.625);// Measured a second time looked like 25 and 5/8 inches but maybe negligible
        public static final Distance ROBOT_HEIGHT = Inches.of(35); // TBA
    }

    public static class Simulation {
        
    }
}

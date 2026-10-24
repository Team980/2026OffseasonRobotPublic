package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED(0),
        EXTEND_HOPPER(2),
        HANGING(6),
        HUNG(0.2), //probably need to remeasure later all of them
        //80 to distance around
        //-4 up
        DOWN(0),
        // UP(-84);
        UP(-25);
        private final double inches;
        //up to down is from -4 to -91 so give some leeway
        //now 0 to 75
        private Position(double inches) {
            this.inches = inches;
        }

        public double get()
        {
            return inches;
        }
        

        public Angle motorAngle() {
            final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
            return Rotations.of(angleMeasure.in(Rotations)); // Promote from Measure<AngleUnit> to Angle
        }
    }

    private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));
    private static final Distance kExtensionTolerance = Inches.of(1);
    private final DutyCycleEncoder throughBoreEncoder;
    public final TalonFX motor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private boolean isHomed = false;
    private final double absBottom = 0.9;
  //  private final double absTop    = 0.7;
    //10 to one gear ratio
    //3.6 absolute at top and 1433 at top
    //8.2 (looped once) and 1367 at bottom
    //calculated difference was to go down takes 14.6 absolute encoder rotations
    //relative was 66 rotations 
    //now adjusting range to 20
    //bottom now at 16.4 relative at -4456.5
    //looped down around at 6.55 at -4390 at top
    //now around 29.85 absolute rotations and again 66.5
    //again starting at top (range is 20)
    //abs: 6.4 (.34), relative at 1529.7 (.69)
    // went down looped twice to 10  and at 1610.168212890625
    //-36.4 absolute rotations and 80.5
    //range at one (inverted is true) starting at bottom to up
    // 0.476 abs and -23.626 relative 0.476 -> 0 -> 1 -> 0 -> 1 -> 0.67
    // passed 0 twice going down and now at 0.67 and relative is -103.84
    //    -1.806 abs                 relative change is -80.214
    // this is from bottom to up
    public Hanger() {
        motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);
        throughBoreEncoder = new DutyCycleEncoder(0, 1, 0);
        throughBoreEncoder.setInverted(true);
       // throughBoreEncoder.setAssumedFrequency(absBottom);
        // double abs = throughBoreEncoder.get();

        // double normalized = normalizeAbs(abs);
        //     //check when less than 0.5 when starts
        //     //decreases as it goes up
        //     //if greater than 0 and less than 0.5
        //     //assume that it has the right position and that bottom
        //     //but if it starts slightly above the start
        //     // then divide by 1.834 or the total range
        //     //to find the percentage its along the total path
        //     //multiply by the total hangar extension range
        //     //set percentage value of 80.214 to it's current position


        // //this at the lower percentage range starting at the bottom
        // double extensionRotation = normalized * 80.214;//the range to rotate relative
        // 10
       // motor.setPosition(extensionRotation);
        double abs = throughBoreEncoder.get();

        // distance from bottom in ABS rotations
        double deltaAbs = abs - absBottom;

        // handle wrap (THIS is the important part)
        if (deltaAbs > 0) deltaAbs -= 1;  // because going "up" decreased
        // now deltaAbs should be ~[-1.806, 0]
        //at relative? -84.2958984375
        // normalize
        //abs currently greater than
        //ran comamnd at the top knowing it should be at the top
        //can check 
        double percent = deltaAbs / -1.806; // 0 → 1

        double motorRotations = percent * -80.214;

        motor.setPosition(motorRotations);
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(20))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    //70
                    .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed.div(4).times(3))//TODO: change to put up KrakenX60.kFreeSpeed
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second).div(4).times(3))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(10)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)/4) // 12 volts when requesting max RPS
            );

        motor.getConfigurator().apply(config);
        //Smartdashboard.putData(this);
    }

      //check when less than 0.5 when starts
            //decreases as it goes up
            //if greater than 0 and less than 0.5
            //assume that it has the right position and that bottom
            //but if it starts slightly above the start
            // then divide by 1.834 or the total range
            //to find the percentage its along the total path
            //multiply by the total hangar extension range
            //set percentage value of 80.214 to it's current position
    // private double normalizeAbs(double abs) {
    //     double value = abs - absBottom;

    //     // unwrap manually
    //     if (value < -0.5) value += 1;
    //     if (value > 0.5) value -= 1;

    //     // now value is in [-0.5, 0.5]
    //     // shift to positive range if needed
    //     if (value < 0) value += 1;

    //     return value / 1.806;
    // }

    //assuming that the abs value is less than 0.5 and greater 0
    //assuming that the climber is near the bottom of the range
     private double normalizeAbs(double abs) {
        double value = abs;
            

        return value / 1.806;
    }

    // private double normalizeAbs(double abs) {
    //     double range = absTop - absBottom;

    //     if (range < 0) {
    //         range += 20; // handle wrap
    //     }

    //     double value = abs - absBottom;

    //     if (value < 0) {
    //         value += 20;
    //     }

    //     return value / range; // 0 → 1
    // }

    

    // public void set(Position position) {
    //     motor.setControl(
    //         motionMagicRequest
    //             .withPosition(position.motorAngle())
    //     );
    // }
    public void set(Position position) {
        motor.setControl(
            motionMagicRequest
                .withPosition(position.get())
        );
    }

    // public void set(Position position) {
    //     motor.setControl(
    //         motionMagicRequest
    //             .withPosition(position.motorAngle())
    //     );
    // }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public Command positionCommand(Position position) {
        return runOnce(() -> set(position))
            .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
    }

    public Command hangarUpCommand()
    {
        return positionCommand(Position.UP);
    }

    public Command hangarDownCommand()
    {
        return positionCommand(Position.DOWN);
    }

    public Command manualUpCommand()
    {
        return run(() -> setPercentOutput(0.08));
    }

    public Command manualDownCommand()
    {
        return run(() -> setPercentOutput(-0.08));
    }

    public Command stopCommand()
    {
        return runOnce(() -> setPercentOutput(0.0));
    }

    // public Command homingCommand() {
    //     return Commands.sequence(
    //         runOnce(() -> setPercentOutput(-0.05)),
    //         Commands.waitUntil(() -> motor.getSupplyCurrent().getValue().in(Amps) > 0.4),
    //         runOnce(() -> {
    //             motor.setPosition(Position.HOMED.motorAngle());
    //             isHomed = true;
    //             set(Position.EXTEND_HOPPER);
    //         })
    //     )
    //     .unless(() -> isHomed)
    //     .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // }

    public boolean isHomed() {
        return isHomed;
    }

    private boolean isExtensionWithinTolerance() {
        final Distance currentExtension = motorAngleToExtension(motor.getPosition().getValue());
        final Distance targetExtension = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
        return currentExtension.isNear(targetExtension, kExtensionTolerance);
    }

    private Distance motorAngleToExtension(Angle motorAngle) {
        final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure.in(Inches)); // Promote from Measure<DistanceUnit> to Distance
    }

    private Angle extensionToMotorAngle(double inches) {
    return Rotations.of(
        Inches.of(inches)
            .divideRatio(kHangerExtensionPerMotorAngle)
            .in(Rotations)
    );
}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Extension (inches)", () -> motorAngleToExtension(motor.getPosition().getValue()).in(Inches), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(
            "Abs Encoder",
            () -> throughBoreEncoder.get(),
            null
        );
        builder.addDoubleProperty(
            "Relative Encoder",
            () -> motor.getPosition().getValueAsDouble(),
            null
        );
    }
}

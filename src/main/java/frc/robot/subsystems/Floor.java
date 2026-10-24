package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.5);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }

        public double outputFraction() {
            return percentOutput;
        }
    }

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
     private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
     private static final AngularVelocity kFeedVelocity = RPM.of(300);
     private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    public Floor() {
        motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(100))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.3)
                    .withKI(0)
                    .withKD(0.0)
                    .withKS(0.3) //was nonthing below 0.2656, 0.117
                    .withKV(0.12) // 12 volts when requesting max RPS
            );

        motor.getConfigurator().apply(config);
        //Smartdashboard.putData(this);
    }

    public void set(Speed speed) {
        setPercentOutput(speed.outputFraction());
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }
    public void set(AngularVelocity speed) {
        motor.setControl(
            velocityRequest
                    .withVelocity(speed)
        );
    }
    // public Command feedCommand() {
    //     return runOnce(() -> set(Speed.FEED));
    // }

    public Command feedCommand() {
        return runOnce(() -> set(kFeedVelocity));
    }

    // public Command feedCommand() {
    //     return runOnce(() -> set(Speed.FEEDSPEED.rpm()))
    //         .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    // }

    public Command stopFloor()
    {
        return runOnce(() -> set(Speed.STOP));
    }

    public boolean isVelocityWithinTolerance() {
        if (!(motor.getAppliedControl() instanceof VelocityVoltage)) return false;
        final AngularVelocity currentVelocity = motor.getVelocity().getValue();
        final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
        return currentVelocity.isNear(targetVelocity, kVelocityTolerance);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}

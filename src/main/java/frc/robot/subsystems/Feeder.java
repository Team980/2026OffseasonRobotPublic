package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(1980),//was 500 //1650 //2000 pretty good
        STOP(0);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }

        public double percent()
        {
            return rpm;
        }
    }

    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    public Feeder() {
        motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(110))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    //50
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.30) //0.4 //ok still oscillation at 0.1
                    .withKI(0)
                    .withKD(0.00)
                    .withKS(0.25) //was nonthing below 0.10345, 0.11
                    .withKV(0.11) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
        //Smartdashboard.putData(this);
    }

    public void set(Speed speed) {
        motor.setControl(
            velocityRequest
                .withVelocity(speed.angularVelocity())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput*12))
        );
    }

    // public Command feedCommand() {
    //     return startEnd(() -> setPercentOutput(Speed.FEEDP.percent()), () -> setPercentOutput(0));
    // }
    public Command feedCommand() {
        return runOnce(() -> set(Speed.FEED));
    }

    // public Command feedCommand() {
    //     return runOnce(() -> set(Speed.FEED))
    //         .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    // }

    public Command stopFeed()
    {
        return runOnce(() -> setPercentOutput(Speed.STOP.percent()));
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

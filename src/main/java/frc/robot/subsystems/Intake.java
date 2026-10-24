package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.8);
        //STALL(1);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;//TODO: change to percentOutput/4 for testing
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }

        public double get()
        {
            return percentOutput;
        }
    }


    public enum Position {
        // HOMED(110),
        
        // STOWED(100), //-1, try -2
        // INTAKE(-4),
        AGITATE(-80),//-65),//-53), //-10
        ReadyToIntake(-108), // -110 if want (PREVIOUSY)
        Start(-3);//can be zero if wants


        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }

        public double value()
        {
            return degrees;
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    public final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private boolean isHomed = false;

    public Intake() {
       // pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kRoboRioCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
        configurePivotMotor();
        configureRollerMotor();
        ////Smartdashboard.putData(this);
    }
    
    private double stallCounter = 0;

    @Override
    public void periodic() {
        if(stallCounter > 5)
        {
            pivotMotor.stopMotor();
            System.out.println("Pivot motor stalled, stopping motor to prevent damage");
        }
        double velocity = pivotMotor.getVelocity().getValueAsDouble();
        double current = pivotMotor.getStatorCurrent().getValueAsDouble();
        double output = pivotMotor.getDutyCycle().getValueAsDouble();

        //Smartdashboard.putNumber("Collector Velocity", velocity);
        //Smartdashboard.putNumber("Collector Current", current);

        boolean tryingToMove = Math.abs(output) > 0.2;
        boolean lowVelocity = Math.abs(velocity) < 1;
        boolean highCurrent = current > 40;

        if (tryingToMove && lowVelocity && highCurrent) {
            stallCounter++;
        } else {
            stallCounter = 0;
        }

        boolean stalled = stallCounter > 5;

        //Smartdashboard.putBoolean("Collector Stalled", stalled);
        

    }


    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)//seems to not be counter clockwise
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))//was 120
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    //70
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()//.div(4).times(3)
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)//TODO: move back up
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(100)//300)//250) //300
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);

        pivotMotor.setPosition(0.0);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    //70
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    public boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    public void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void set(Position position) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
        
    }

    public void setAdjusted(Position position, double adjustmentDegrees) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle().minus(Degrees.of(adjustmentDegrees)))
            );
    }


    // public void set(Speed speed) {
    //     rollerMotor.setControl(
    //         rollerVoltageRequest
    //             .withOutput(speed.voltage())
    //     );
    // }
    public void set(double percent) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(percent * 12.0) //12 volts
        );
    }
    
    
    public Command percentMoveCommand(double percentOutput) {
        // System.out.println(pivotMotor.getSupplyCurrent().getValue().in(Amps));
        return startEnd(() ->
                 setPivotPercentOutput(percentOutput),
            () -> setPivotPercentOutput(0));
    }

    // public Command intakeCommand() {
    //     return startEnd(
    //         () -> {
    //             set(Position.ReadyToIntake);
    //             set(Speed.INTAKE.get());
    //         },
    //         () -> set(Speed.STOP.get())
    //     );
    // }

    public Command intakeCommand() {
        return run(
            () -> {
                set(Speed.INTAKE.get());
            });
    }

    public Command stopRollers()
    {
        return runOnce(() -> set(Speed.STOP.get()));
    }

    public Command goOut() {
        return runOnce(() -> {
            set(Speed.STOP.get());
            set(Position.ReadyToIntake);
        });
    }

    public void resetDeployEncoder(){
        pivotMotor.setPosition(0);
    }

    public void resetDeployEncoderDown(){ // TODO: NOT WORKING
        pivotMotor.setPosition(Position.ReadyToIntake.angle().in(Rotations));
    }

    public void stopMotor()
    {
        pivotMotor.stopMotor();
    }

    public Command returnIn()
    {
        return runOnce(() -> {
            set(Speed.STOP.get());
            set(Position.Start);
        });
    }
    //Commands.Wait(seconds) if extended time
    public Command agitateCommand() {
        return runOnce(() -> set(Speed.STOP.get()))//Speed.intake.get()
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.ReadyToIntake)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.ReadyToIntake);
                set(Speed.STOP.get());
            });
    }

    //ran till hardstop with amp check
    // public Command homingCommand() {
        
    //     return Commands.sequence(
    //         runOnce(() -> setPivotPercentOutput(0.1)),
    //         //Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
    //         Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 0
    //         && pivotMotor.getSupplyCurrent().getValue().in(Amps) < 0.25),
    //         runOnce(() -> {
    //             pivotMotor.setPosition(Position.HOMED.angle());
    //             isHomed = true;
    //             set(Position.STOWED);
    //         })
    //     )
    //     .unless(() -> isHomed)
    //     .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Pivot Stator Current", () -> pivotMotor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Stator Current", () -> rollerMotor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("TEST Test", () -> 0.31415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679, null);
    }
}

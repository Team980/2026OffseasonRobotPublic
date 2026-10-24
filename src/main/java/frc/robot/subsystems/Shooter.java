package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Landmark;
import frc.robot.Ports;
import frc.robot.RobotContainer;
import frc.util.TargetUtil;

public class Shooter extends SubsystemBase {
    public final Time maxDelay = Seconds.of(1);
    public final Time revTime = Seconds.of(3);
    public final Time stableTime = Seconds.of(0.5);
    public boolean everyThingGood = false;
    public double stableTimeCounter = -1;
    public DoublePublisher timePublisher = NetworkTableInstance.getDefault().getDoubleTopic("SmartDashboard/shot timing/stable time").publish();
    public BooleanPublisher rpmTolerance = NetworkTableInstance.getDefault().getBooleanTopic("SmartDashboard/shot timing/shooter rpm within tolerance").publish();
    public BooleanPublisher hoodTolerance = NetworkTableInstance.getDefault().getBooleanTopic("SmartDashboard/shot timing/hood position within tolerance").publish();
    public BooleanPublisher stablePublisher = NetworkTableInstance.getDefault().getBooleanTopic("SmartDashboard/shot timing/stable and aiming and hood good").publish();
    public Double prevTime = Timer.getFPGATimestamp();
        
    private static final AngularVelocity kVelocityTolerance = RPM.of(200);

    public final TalonFX leftMotor, middleMotor, rightMotor;
    public final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 3000;
    private static final double idleRPM = 300; // 500 // could also be 1000// was 0.0 //3000
    //3000 only shot 
    // 1-2 feet in the air not high enough for hub
    //6000 shoot full power through the roof greater than 10 feet in the air

    public Shooter() {

        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.Clockwise_Positive);
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(rightMotor, InvertedValue.CounterClockwise_Positive);
        /*   configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);//
        configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive); */
        //Smartdashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(140))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(120))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs() //0.5 //0.8
                    .withKP(0.8)//0.85)//0.75) //0.68) // was 0.5 moving to 0.75
                    .withKI(0)
                    .withKD(0.008) 
                    .withKS(0.21)//was 0? //was 0.11
                    .withKV(0.125)//0.145) // 12 volts when requesting max RPS//was 12/Krakenx60maxSpeed
                    //0.14
            );
        
        motor.getConfigurator().apply(config);
    }
    @Override
    public void periodic() {
        boolean rpmGood = isVelocityWithinTolerance();
        boolean hoodGood = Hood.withinTolerance;
        hoodTolerance.set(hoodGood);

        if (rpmGood & hoodGood) {
            if (stableTimeCounter == -1)
                prevTime = Timer.getFPGATimestamp();
            stableTimeCounter = Timer.getFPGATimestamp() - prevTime;
        } else {
            stableTimeCounter = -1;
        }

        boolean timeGood = stableTimeCounter > stableTime.in(Seconds);

        if (rpmGood && hoodGood && timeGood) {
            stablePublisher.set(true);
            everyThingGood = true;
        } else {
            stablePublisher.set(false);
            everyThingGood = false;
        }

        timePublisher.set(stableTimeCounter);

        rpmTolerance.set(isVelocityWithinTolerance());
    }
    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
            //  motor.setVoltage(controller.calculate(motor.getVelocity().getValueAsDouble(),
            //     rpm) *12 + 0.9 * motor.getClosedLoopFeedForward().getValueAsDouble());
        }
    }
    
    public Command idleShooterCommand() {
        return run(()-> setRPM(idleRPM));
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput*12))
            );
        }
    }

    public boolean isWithinThresholdAndStableAndAiming(Swerve swerve) {
        return everyThingGood && TargetUtil.isAiming(swerve);
    }

    public boolean isWithinThresholdAndStable() {
        return everyThingGood;
    }

    public void stop() {
        setRPM(idleRPM);
    }

    public void eStop(){
        setPercentOutput(0);
    }

    public Command stopShootCommand() {
        return runOnce(() -> setPercentOutput(0.0));
    }

    public void slowSpeed()
    {
        setPercentOutput(0.5);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public static DoublePublisher currentRpm = NetworkTableInstance.getDefault().getDoubleTopic("SmartDashboard/shooter tests/shooter current rpm").publish();
    public static DoublePublisher targetRpm = NetworkTableInstance.getDefault().getDoubleTopic("SmartDashboard/shooter tests/shooter target rpm").publish();
    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            currentRpm.set(currentVelocity.in(RPM));
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            targetRpm.set(targetVelocity.in(RPM));
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
        builder.addBooleanProperty("Reached Velocity Tolerance", () -> isVelocityWithinTolerance(), null);
    }
}

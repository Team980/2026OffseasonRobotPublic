package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.signals.RobotEnableValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Robot;

public class Hood extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20/2).per(Second);//TODO: Multiply by 2
    public static final double kMinPosition = 0.01;
    public static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;
    private static  double kStepSize = 0.02;
    public static boolean withinTolerance = false;

    private final Servo leftServo;
    private final Servo rightServo;

    // private final LinearServo leftLinearServo;
    // private final LinearServo rightLinearServo;

    public double currentPosition = 0.5;
    private double targetPosition = 0.5;
    private double speed = 1.0;
    private Time lastUpdateTime = Seconds.of(0);

    public Hood() {
        leftServo = new Servo(Ports.kHoodLeftServo);
        rightServo = new Servo(Ports.kHoodRightServo);

        // leftLinearServo = new LinearServo(Ports.kHoodLeftServo, 100, 10);
        // rightLinearServo = new LinearServo(Ports.kHoodRightServo, 100, 10);


        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        // leftLinearServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        // rightLinearServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setPosition(currentPosition);
        ////Smartdashboard.putData(this);
        ////Smartdashboard.putNumber("FPGA Timestamp", Timer.getFPGATimestamp());
        //test later to make sure whether this is needed
        //lastUpdateTime = Seconds.of(Timer.getFPGATimestamp());
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
        // leftServo.setPosition(clampedPosition);
        // rightServo.setPosition(clampedPosition);
      
        targetPosition = clampedPosition;
    }

    // public void setPosition(double position)
    // {
    //     leftLinearServo.setPosition(position);
    //     rightLinearServo.setPosition(position);
    // }

    

    public void moveUp()
    {
        kStepSize = 0.05;
        setPosition(targetPosition + kStepSize);
    }
    public void moveDown()
    {
        kStepSize = -0.05;
        setPosition(targetPosition + kStepSize);
    }   

    public void stopMove()
    {
        kStepSize = 0;
    }
    //  public void moveUp()
    // {
    //     leftServo.setSpeed(0.05);
    //     rightServo.setSpeed(0.05);
    // }
    // public void moveDown()
    // {
    //     leftServo.setSpeed(-0.05);
    //     rightServo.setSpeed(-0.05);
    // }   

    // public void stopMoving()
    // {
    //     leftServo.setSpeed(0.0);
    //     rightServo.setSpeed(0.0);
    // }

    boolean buttonPressed = false;
    public Command moveUpCommand() {
        //buttonPressed = true;
        return Commands.run(() -> moveUp());
       // return Commands.run(this::moveUp); // used to be return Commands.run(this::moveUp)
    }
    public Command moveDownCommand() {
        return Commands.run(() -> moveDown()); // used to be return Commands.run(this::moveDown)
    }

    public Command stopMoveCommand()
    {
        return Commands.runOnce(() -> stopMove());
    }



    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        ////Smartdashboard.putNumber("Hood current", currentPosition);
        ////Smartdashboard.putNumber("Hood target", targetPosition);

        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
        Hood.withinTolerance = isPositionWithinTolerance();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
//import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;
import frc.util.TargetUtil;
import frc.robot.*;

public class AimAndDriveCommand extends Command {
    private static Translation2d prevTarget;

    private final Swerve swerve;
    private final ManualDriveCommand manualDriveCommand;
    private final Supplier<Translation2d> target;

    // private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6.0);

    public AimAndDriveCommand(
        Swerve swerve,
        ManualDriveCommand manualDriveCommand,
        Supplier<Translation2d> target
    ) {
        this.swerve = swerve;
        this.target = target;
        this.manualDriveCommand = manualDriveCommand;
    }

    public AimAndDriveCommand(Swerve swerve, ManualDriveCommand manualDriveCommand) {
        this(swerve, manualDriveCommand, () -> TargetUtil.getTarget(swerve));
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, RobotContainer.manualDriveCommand);
    }

   
    private Rotation2d getDirectionToTarget() {
        final Translation2d targetPosition = target.get();//Alliance.Red == Robot.alliance ? 
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();

        final Rotation2d targetDirectionInBlueAlliancePerspective = targetPosition.minus(robotPosition).getAngle();
        final Rotation2d targetDirectionInOperatorPerspective = targetDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        //SmartDashboard.putNumber("Angle to Target (deg)", targetDirectionInBlueAlliancePerspective.getDegrees());
        return targetDirectionInOperatorPerspective;
    }

    public Rotation2d getTargetAngle() {
        return getDirectionToTarget();
    }

    @Override
    public void execute() {
        if (!swerve.isAiming) {
            swerve.isAiming = true;
        }
        // final ManualDriveInput input = inputSmoother.getSmoothedInput();
        Rotation2d targetDirection = getDirectionToTarget();
        // targetAngle = targetDirection.getDegrees();
        // swerve.setControl(
        //     fieldCentricFacingAngleRequest
        //         .withVelocityX(Driving.kMaxSpeed.times(input.forward))
        //         .withVelocityY(Driving.kMaxSpeed.times(input.left))
        //         .withTargetDirection(getDirectionToTarget())
        manualDriveCommand.setLockedHeading(targetDirection);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    

    @Override
    public void end(boolean interrupted) {
        swerve.isAiming = false;
    }
}
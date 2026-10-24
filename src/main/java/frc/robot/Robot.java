// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import frc.util.CustomJoystick;
import frc.util.TargetUtil;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControllerSetting;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.util.CustomJoystick;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;
    private Command autonomousCommand;
    public BooleanPublisher isEnabled = NetworkTableInstance.getDefault().getBooleanTopic("SmartDashboard/robot states/is enabled").publish();
    public BooleanPublisher isTeleop = NetworkTableInstance.getDefault().getBooleanTopic("SmartDashboard/robot states/is teleop").publish();
    public DoublePublisher matchTimePub = NetworkTableInstance.getDefault().getDoubleTopic("SmartDashboard/time/match time").publish();
    public DoublePublisher countdownPub = NetworkTableInstance.getDefault().getDoubleTopic("SmartDashboard/time/count down").publish();
    public double prevTime = -1;
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        System.out.println("Constants.currentMode: " + Constants.currentMode);  
        switch (Constants.currentMode) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                //Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                RobotController.setBrownoutVoltage(6.1);
            }

            case SIM -> {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            }

            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }
        Logger.start();

        m_robotContainer = new RobotContainer();
        //////////Smartdashboard.putData(CommandScheduler.getInstance());
        RobotController.setBrownoutVoltage(Volts.of(6.1));
    }

    @Override
    public void disabledPeriodic()
    {
        // Vision seeding is re-armed once in disabledInit().
    }

    @Override
    public void disabledInit(){
        m_robotContainer.seedAgain();
        m_robotContainer.shooter.eStop();
        //m_robotContainer.subsystemCommands.resetRobot();
    }
    
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        matchTimePub.set(DriverStation.getMatchTime());
        isEnabled.set(DriverStation.isEnabled());
        isTeleop.set(DriverStation.isTeleop());
        // start at high priority for robot code to minimize latency
        Threads.setCurrentThreadPriority(true, 99);
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(false, 10);
        m_robotContainer.updateSwerveTelemetry();
        Logger.recordOutput("Zeroed exterior poses", new Pose3d [] {new Pose3d()});
        Logger.recordOutput("ZeroedInteriorPoses", new Pose3d[] {new Pose3d()});
        Logger.recordOutput(
        "FinalCompoenentPoses",
        new Pose3d[] {
          new Pose3d(
              0, 0, Math.sin(Timer.getTimestamp()) * 0.37 - 0.07, new Rotation3d(0.0, 0.0, 0.0))
        });

        // AdvantageScope via Advantagkit trying to log the motor positions in 3d field
        //Intakeangle
        // Pose2d robotPose =
        //     m_robotContainer.swerve.getState().Pose;
        // double intakeAngle = 
        //     m_robotContainer.intake.pivotMotor.getPosition().getValueAsDouble();
        // double climberPos =
        //     m_robotContainer.hanger.motor.getPosition().getValueAsDouble();
        // Logger.recordOutput(
        //     "Robot Mechanisms",  new Pose3d[] {

        // // Intake pivot
        // new Pose3d(
        //     robotPose.getX() + 0.25,
        //     robotPose.getY(),
        //     0.15,
        //     new Rotation3d(
        //         0,
        //         intakeAngle,
        //         robotPose.getRotation().getRadians()
        //     )),
        // new Pose3d(
        //     robotPose.getX(),
        //     robotPose.getY(),
        //     climberPos,
        //     new Rotation3d(
        //         0,
        //         0,
        //         robotPose.getRotation().getRadians()
        //     )
        // )
        // });
            
            // Shooter angle
      //  m_robotContainer.swerve.updateSimState(defaultPeriodSecs, defaultPeriodSecs);

        //System.out.println(m_robotContainer.swerve.getState().Pose);
       // System.out.println(m_robotContainer.driver.getRawAxis());
    //    var state = m_robotContainer.swerve.getState();

    //     var flTarget = state.ModuleTargets[0];
    //     var flActual = state.ModuleStates[0];

    //     //////////Smartdashboard.putNumber("FL Target Speed",
    //         flTarget.speedMetersPerSecond);

    //     //////////Smartdashboard.putNumber("FL Actual Speed",flActual.speedMetersPerSecond);
    //     //////////Smartdashboard.putNumber("FL Speed Error", flTarget.speedMetersPerSecond - flActual.speedMetersPerSecond);
    //     var constrainTargetAngle = ((flTarget.angle.getDegrees() + 180) % 360 + 360) % 360 - 180;
    //     var constrainActualAngle = ((flActual.angle.getDegrees() + 180) % 360 + 360) % 360 - 180;
    //     //////////Smartdashboard.putNumber("FL Target Angle",
    //         constrainTargetAngle);

    //     //////////Smartdashboard.putNumber("FL Actual Angle", constrainActualAngle);

    //     //////////Smartdashboard.putNumber("FL Angle Error", MathUtil.angleModulus(constrainTargetAngle - constrainActualAngle));

       //System.out.println(TunerConstants.FrontLeft.)
       //System.out.println(TargetUtil.targetOffset);
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        prevTime = Timer.getFPGATimestamp();
       autonomousCommand = m_robotContainer.getAutonomousCommand();

        //schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic()
    {
        countdownPub.set(20-((int) ((Timer.getFPGATimestamp() - prevTime) * 100) / 100));
    }

    @Override
    public void teleopPeriodic() {
        double dt = (int) ((Timer.getFPGATimestamp() - prevTime) * 100) / 100;
        double res = -1;
        if (dt < 10) res = 10 - dt;
        else if (dt < 35) res = 35 - dt;
        else if (dt < 60) res = 60 - dt;
        else if (dt < 85) res = 85 - dt;
        else if (dt < 110) res = 110 - dt;
        else res = 140 - dt;
        countdownPub.set(res);
        // SignalLogger.writeValue("SwerveMotorVelocity",
        //  RotationsPerSecond.of(m_robotContainer.swerve.getModule(0).getSteerMotor().getVelocity().getValueAsDouble()));
        //  SignalLogger.writeValue("SwerveMotorPosition",
        //  RotationsPerSecond.of(m_robotContainer.swerve.getModule(0).getSteerMotor().getPosition().getValueAsDouble()));
        //   SignalLogger.writeValue("SwerveMotorVoltage",
        //  Volts.of(m_robotContainer.swerve.getModule(0).getSteerMotor().getMotorVoltage().getValueAsDouble()));
    }

    @Override
    public void teleopInit() {
        prevTime = Timer.getFPGATimestamp();
        if (Constants.ControllerSettingManuallySet == null) {
            if (DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1)) {
                RobotContainer.controllerSetting = ControllerSetting.TWO;
            } else {
                RobotContainer.controllerSetting = ControllerSetting.ONE;
            }
        } else {
            RobotContainer.controllerSetting = Constants.ControllerSettingManuallySet;
        }

        if (RobotContainer.controllerSetting == ControllerSetting.TWO) {
            RobotContainer.driverShiftKey = false;
        }

        RobotContainer.controllerSettingPublisher.set(RobotContainer.controllerSetting == ControllerSetting.TWO ? "two" : "one");

        CommandScheduler.getInstance().schedule(RobotContainer.aimOffsetCommand);
    }

    private boolean testBlue = false;

    @Override
    public void simulationInit() {
        // if (testBlue) {
        //     DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        //     DriverStationSim.notifyNewData(); 
        // }
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        m_robotContainer.swerve.updateSimState(
            0.02,
            RobotController.getBatteryVoltage()
        );

        CustomJoystick.updateAllJoysticksTelemetry();
    }
}

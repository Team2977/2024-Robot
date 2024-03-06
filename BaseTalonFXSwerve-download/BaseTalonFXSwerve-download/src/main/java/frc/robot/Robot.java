// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;







import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.UpperAssembly.moveShoulder;
import frc.robot.subsystems.intake;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
 // private vision Vision;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
  
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  public static double xSpeed;
  public static double ySpeed;
  public static Optional<Alliance> alliance = DriverStation.getAlliance();
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SignalLogger.stop();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Constants.driveSpeed = 1;
    Constants.turnSpeed = 1;
    Constants.wantedShoulderAngle = 0;
    Constants.flywheelSpeed = 0;
    Constants.slowMode = true;
    Constants.wantedClimberPose = 0;
    
    moveShoulder.shoulderPID.reset();
    intake.shoulder.setNeutralMode(NeutralModeValue.Brake);
    intake.leftHook.setPosition(0);
    intake.rightHook.setPosition(0);

    

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

   
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    intake.shoulder.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
        SignalLogger.stop();
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      Constants.invert = 1;
      Constants.onRedTeam = true;
      Constants.wantedApriltag = 4;
    } else {
      Constants.invert = -1;
      Constants.onRedTeam = false;
      Constants.wantedApriltag = 7;
    }
    intake.leftHook.setNeutralMode(NeutralModeValue.Brake);
    intake.rightHook.setNeutralMode(NeutralModeValue.Brake);
    Constants.autoDriveMode = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Constants.autoDriveMode = false;
    intake.rightIntake.set(0);
    intake.leftIntake.set(0);
    intake.shooter.set(0);
    intake.shooterSlave.set(0);
    intake.indexer.set(0);
    

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      Constants.invert = 1;
      Constants.onRedTeam = true;
      Constants.wantedApriltag = 4;
    } else {
      Constants.invert = -1;
      Constants.onRedTeam = false;
      Constants.wantedApriltag = 7;
    }
        SignalLogger.stop();
    intake.leftHook.setNeutralMode(NeutralModeValue.Brake);
    intake.rightHook.setNeutralMode(NeutralModeValue.Brake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //this gets the x and y values for the command "turnToTarget"
    xSpeed = MathUtil.applyDeadband(RobotContainer.driver.getRawAxis(1) / Constants.driveSpeed * Constants.invert, Constants.stickDeadband);
    ySpeed = MathUtil.applyDeadband(RobotContainer.driver.getRawAxis(0) / Constants.driveSpeed * Constants.invert, Constants.stickDeadband);

    SmartDashboard.putBoolean("on red team", Constants.onRedTeam);
    SmartDashboard.putString("auto mode for path", SmartDashboard.getData("Auto Mode").toString());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    intake.leftHook.setNeutralMode(NeutralModeValue.Coast);
    intake.rightHook.setNeutralMode(NeutralModeValue.Coast);
  }
}

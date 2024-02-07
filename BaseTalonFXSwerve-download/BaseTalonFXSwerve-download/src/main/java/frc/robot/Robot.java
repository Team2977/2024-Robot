// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;






import java.lang.annotation.Target;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.TargetCornerProto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


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
  public static DutyCycleEncoder shoulderPos = new DutyCycleEncoder(0);
  
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  private static double yaw;
  private PhotonTrackedTarget lastTarget;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Constants.driveSpeed = 1;
    Constants.turnSpeed = 1;
    //Vision = new vision();
  // PortForwarder.add(5800, "10.29.77.11", 5800);
    //get camera name
   

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

   /*  var result = camera.getLatestResult();

    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var yaw = target.getYaw();
      var pitch = target.getPitch();
      var camTotarget = target.getBestCameraToTarget();
    }*/

    /*  // Correct pose estimate with vision measurements
     var visionEst = Vision.getEstimatedGlobalPose();
     visionEst.ifPresent(
             est -> {
                 var estPose = est.estimatedPose.toPose2d();
                 // Change our trust in the measurement based on the tags we can see
                 var estStdDevs = Vision.getEstimationStdDevs(estPose);

                 RobotContainer.s_Swerve.addVisionMeasurement(
                         est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
             });*/
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    SmartDashboard.putData("encoder", shoulderPos);
    
    
    var phoResu = RobotContainer.photonCamera.getLatestResult();
    if(phoResu.hasTargets()) {
      var targetOpt = phoResu.getTargets().stream()
          .filter(t -> t.getFiducialId() == 4)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
       yaw = targetOpt.get().getYaw().getasdouble();
      // targetOpt = phoResu.getTargets().stream().filter();
    } else {
      yaw = 0;
    }

    SmartDashboard.putNumber("RposeX", RobotContainer.s_Swerve.swerveOdometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("RposeY", RobotContainer.s_Swerve.swerveOdometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("yaw", yaw);
    //SmartDashboard.putNumber("Robot x", this.Vision.getEstimatedGlobalPose().get());
   

    //intake.shoulder.set(RobotContainer.gamepad2.getRawAxis(1)/10);

    
    SmartDashboard.putNumber("7poseX", RobotContainer.poseESTIMATOR.getCurrentPose().getX());
    SmartDashboard.putNumber("7poseY", RobotContainer.poseESTIMATOR.getCurrentPose().getY());
    

    
    
    //SmartDashboard.putNumber("tag #", );

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

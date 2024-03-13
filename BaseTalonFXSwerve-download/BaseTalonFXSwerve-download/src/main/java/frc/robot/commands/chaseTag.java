// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;



import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.Swerve;

public class chaseTag extends Command {

  private PhotonCamera photonCamera;
  private Swerve swerve;
  private Supplier<Pose2d> poseProvider;


  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 4);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 4);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.degreesToRadians(500), Units.degreesToRadians(500));
  //private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.Swerve.maxAngularVelocity, Units.feetToMeters(3));

  private static final int TagToChase = 4;
  private static final Transform3d TagToGoal = 
                new Transform3d(
                                  new Translation3d(2, 0, 0),
                                  new Rotation3d(0, 0, 0/*Math.PI*/));

  private final ProfiledPIDController xController = new ProfiledPIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =  new ProfiledPIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, OMEGA_CONSTRAINTS);
  
  private PhotonTrackedTarget lastTarget;

  /** Creates a new chaseTag. */
  public chaseTag(
    PhotonCamera photonCamera,
    Swerve swerve,
    Supplier<Pose2d> poseProvider) {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(swerve);
      this.photonCamera = photonCamera;
      this.swerve = swerve;
      this.poseProvider = poseProvider;

      xController.setTolerance(0.2);
      yController.setTolerance(0.2);
      omegaController.setTolerance(Units.degreesToRadians(3));
      omegaController.enableContinuousInput(-Math.PI, Math.PI);
  
      
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = this.poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TagToChase)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(Constants.Vision.kRobotToCam);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TagToGoal).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget == null) {
      // No target has been visible
      this.swerve.drive(new Translation2d(0,0), 0, false, true);
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      


       this.swerve.drive(
        new Translation2d(xSpeed, ySpeed)/* .times(Constants.Swerve.maxSpeed)*/,
        omegaSpeed /* * Constants.Swerve.maxAngularVelocity/Constants.turnSpeed*/,
        //field centric
        false,
        true
        );
    }
      
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

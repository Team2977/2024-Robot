// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.poseEstimator;



public class turnToTarget extends Command {
  /** Creates a new turnToTarget. */

  private Swerve swerve;
  private PhotonCamera photonCamera;
  private double rotationValue;
  private Supplier<Pose2d> poseProvider;
  private poseEstimator poseSubsystem;

  private List<PhotonTrackedTarget> targets;
  private int targetID;
  private double poseX;
  private double poseY;

  //change omega constraints
  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.feetToMeters(8), Units.feetToMeters(6));

  private final PIDController pidControllerX = new PIDController(0.1, 0, 0);
  private final PIDController pidControllerY = new PIDController(0.1, 0, 0);
  private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(1, 0.5, 0, omegConstraints);

  public turnToTarget(Swerve swerve, PhotonCamera photonCamera, Supplier<Pose2d> poseProvider, poseEstimator poseEstimator) {
    this.swerve = swerve;
    this.photonCamera = photonCamera;
    this.poseProvider = poseProvider;
    this.poseSubsystem = poseEstimator;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("turning", true);
    pidControllerOmega.reset(poseProvider.get().getRotation().getRadians());
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);
    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



   var omegaSpeed = pidControllerOmega.calculate(poseSubsystem.getTargetYaw(Constants.wantedApriltag), swerve.getHeading().getRadians());
    if (pidControllerOmega.atGoal()) {
      omegaSpeed = 0;
    }
    
    swerve.drive(
                  new Translation2d(-Robot.xSpeed, -Robot.ySpeed).times(Constants.Swerve.maxSpeed), 
                  (omegaSpeed / Constants.turnSpeed) * Constants.Swerve.maxAngularVelocity, 
                  true, 
                  true
                  );




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.swerve.drive(new Translation2d(), 0, true, true);
    SmartDashboard.putBoolean("turning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

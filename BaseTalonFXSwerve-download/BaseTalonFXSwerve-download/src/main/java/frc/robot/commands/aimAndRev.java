// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.Supplier;

import org.photonvision.PhotonCamera;



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
import frc.robot.subsystems.intake;
import frc.robot.subsystems.poseEstimator;

public class aimAndRev extends Command {
  private final intake intake;
  private final Swerve swerve;
  private final PhotonCamera photoncamera;
  private double translationAxis;
  private double strafeAxis;
  private int wantedApriltag;
  private final Supplier<Pose2d> poseProvider;
  private final poseEstimator poseSubsystem;


  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.feetToMeters(8), Units.feetToMeters(8));
  
  private final PIDController pidControllerX = new PIDController(0.5, 0.1, 0);
  private final PIDController pidControllerY = new PIDController(0.2, 0.05, 0);
  private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(.8, 0.3, 0, omegConstraints);



  public aimAndRev(intake intake, PhotonCamera photonCamera, Swerve swerve, Supplier<Pose2d> poseProvider, poseEstimator poseEstimator) {
    this.intake = intake;
    this.photoncamera = photonCamera;
    this.swerve = swerve;
    this.poseProvider = poseProvider;
    this.poseSubsystem = poseEstimator;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset(poseProvider.get().getRotation().getRadians());

   // pidControllerX.setSetpoint(Units.inchesToMeters(100)); // Move forward/backwork to keep 36 inches from the target
    //pidControllerX.setTolerance(Units.inchesToMeters(5));

    //pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    //pidControllerX.setTolerance(Units.inchesToMeters(2.5));

   // pidControllerOmega.setGoal(Units.degreesToRadians(-90)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putBoolean("Finding target", true);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    var wantedtagPose = poseSubsystem.aprilTagFieldLayout.getTagPose(4).get();
    var wantedTagPose2d = poseSubsystem.aprilTagFieldLayout.getTagPose(4).get().toPose2d();
    var getDistanceToPose =  PhotonUtils.getDistanceToPose(poseSubsystem.getCurrentPose(), wantedTagPose2d);
    var camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(getDistanceToPose, swerve.getHeading());
    */
    
    var targetYaw = poseSubsystem.getTargetYaw(Constants.wantedApriltag);
    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);

    
    
    //put the equation for finding the ideal angle here. using the target distance
   // Constants.wantedShoulderAngle = 

    var targetX = poseSubsystem.getTargetPose2d(Constants.wantedApriltag).getX();
    var targetY = poseSubsystem.getTargetPose2d(Constants.wantedApriltag).getY();
    var robotX = poseProvider.get().getX();
    var robotY = poseProvider.get().getY();

    var sideX = robotX - targetX;
    var sideY = robotY - targetY;
  
    var angle = Math.atan(sideY / sideX);
    
   var omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians(), angle);
    if (pidControllerOmega.atGoal()) {
      omegaSpeed = 0;
    }
    
    swerve.drive(
                  new Translation2d(-Robot.xSpeed, -Robot.ySpeed).times(Constants.Swerve.maxSpeed), 
                  (omegaSpeed / Constants.turnSpeed) * Constants.Swerve.maxAngularVelocity, 
                  true, 
                  true
                  );

  //  intake.shooter.set(0.9);
   // intake.shooterSlave.set(0.9);
   SmartDashboard.putNumber("omega goal", pidControllerOmega.getGoal().position);
   SmartDashboard.putNumber("wanted angle", angle);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
    frc.robot.subsystems.intake.shooter.set(0);
    intake.shooterSlave.set(0);
    Constants.wantedShoulderAngle = 0;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

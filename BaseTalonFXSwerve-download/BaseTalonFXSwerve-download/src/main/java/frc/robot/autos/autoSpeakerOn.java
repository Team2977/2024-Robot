// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.poseEstimator;

public class autoSpeakerOn extends Command {
  /** Creates a new autoSpeaker. */
  private final intake intake;
  private final Swerve swerve;
  private final PhotonCamera photoncamera;  
  private int wantedApriltag;
 // private final Supplier<Pose2d> poseProvider;
  private final poseEstimator poseSubsystem;
  private double angleOffset;


  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(500));
  
  //private final PIDController pidControllerX = new PIDController(0.5, 0.1, 0);
  //private final PIDController pidControllerY = new PIDController(0.2, 0.05, 0);
  private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(0.1, 0, 0, omegConstraints);

  public autoSpeakerOn(intake intake, PhotonCamera photonCamera, Swerve swerve, poseEstimator poseEstimator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.INTAKE);
    this.intake = intake;
    this.photoncamera = photonCamera;
    this.swerve = swerve;
   // this.poseProvider = poseProvider;
    this.poseSubsystem = poseEstimator;
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    //pidControllerX.reset();
    //pidControllerY.reset();
    pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
   
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);

    Constants.speakerSpeed = 96;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);
    var wantedAngle = poseSubsystem.getAngleToSpeaker();
   
     //more them 4 meters away
  if (targetDistance > 4) {
    angleOffset = Units.degreesToRadians(1);
  } else { angleOffset = Units.degreesToRadians(3);}

   var omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians() - angleOffset, wantedAngle);
    if (pidControllerOmega.atGoal()) {
      omegaSpeed = 0;
    }
    
    swerve.drive(
                  new Translation2d(Robot.xSpeed, Robot.ySpeed).times(Constants.Swerve.maxSpeed), 
                  (omegaSpeed / Constants.turnSpeed) * Constants.Swerve.maxAngularVelocity, 
                  true, 
                  true
                  );

    //Constants.wantedShoulderAngle = 12.7 - (1.04 * targetDistance) - (0.0631 * Math.pow(targetDistance, 2));
    //Constants.wantedShoulderAngle = 18.8 - (3.88 * targetDistance) + (0.368 * Math.pow(targetDistance, 2));
    Constants.wantedShoulderAngle = 7.63
                                   + (10.7 * targetDistance) 
                                   - (7.48 * Math.pow(targetDistance, 2)) 
                                   + (1.8 * Math.pow(targetDistance, 3)) 
                                   - (0.148 * Math.pow(targetDistance, 4));  

   // frc.robot.subsystems.intake.shooter.setControl(intake.vDC.withVelocity(Constants.speakerSpeed));
    //frc.robot.subsystems.intake.shooterSlave.setControl(intake.vDC.withVelocity(Constants.speakerSpeed));
    
/* 
   SmartDashboard.putNumber("omega goal", pidControllerOmega.getGoal().position);
   SmartDashboard.putNumber("robot rota", poseSubsystem.getCurrentPose().getRotation().getRadians());
   SmartDashboard.putNumber("dis to tar", targetDistance);
*/




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
    //Constants.speakerSpeed = 0;
    //intake.shooter.setControl(intake.vDC.withVelocity(0));
    //intake.shooterSlave.setControl(intake.vDC.withVelocity(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.UpperAssembly.shoulderDown;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.poseEstimator;

public class aimAndRev extends Command {
  private final intake INTAKE;
  private final Swerve swerve;
  private final PhotonCamera photoncamera;  
 // private final Supplier<Pose2d> poseProvider;
  private final poseEstimator poseSubsystem;
  private double angleOffset;
  


  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(500));
  private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);



  public aimAndRev(intake intake, PhotonCamera photonCamera, Swerve swerve, poseEstimator poseEstimator) {
    this.INTAKE = intake;
    this.photoncamera = photonCamera;
    this.swerve = swerve;
   // this.poseProvider = poseProvider;
    this.poseSubsystem = poseEstimator;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);
    var wantedAngle = poseSubsystem.getAngleToSpeaker();

  /*   //more them 4 meters away
  if (targetDistance > 4) {
    angleOffset = Units.degreesToRadians(0);
  } else { angleOffset = Units.degreesToRadians(5);}*/
  //angleOffset = 0;
    

  //if(targetDistance <= 3.5) {
    
   var omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians(), wantedAngle);
    if (pidControllerOmega.atGoal()) {
      omegaSpeed = 0;
    }
    //Constants.robotRotationSpeed = omegaSpeed;
   
  swerve.drive(
                  new Translation2d(Robot.xSpeed, Robot.ySpeed).times(Constants.Swerve.maxSpeed), 
                  (omegaSpeed / Constants.turnSpeed) * Constants.Swerve.maxAngularVelocity, 
                  true, 
                  true
                  );


    //first equation. the fallback 
    //Constants.wantedShoulderAngle = 12.7 - (1.04 * targetDistance) - (0.0631 * Math.pow(targetDistance, 2)); //started with 12.5

    //second equation. it shoots high
    //Constants.wantedShoulderAngle = 18.8 - (3.88 * targetDistance) + (0.368 * Math.pow(targetDistance, 2));

    //third equation. R^2 == 0.999
   /*  Constants.wantedShoulderAngle = 14.1 
                                    + (0.0956 * targetDistance) 
                                    - (1.22 * Math.pow(targetDistance, 2)) 
                                    + (0.209 * Math.pow(targetDistance, 3));*/

    //forth equation. same dataset as the first, just to the 4th polynomial.  R^2 = 1
    Constants.wantedShoulderAngle = 7.63
                                   + (10.7 * targetDistance) 
                                   - (7.48 * Math.pow(targetDistance, 2)) 
                                   + (1.8 * Math.pow(targetDistance, 3)) 
                                   - (0.148 * Math.pow(targetDistance, 4))
                                   - 0.4;  


   frc.robot.subsystems.intake.setFlywheelSpeed(96);
  //} 

   
   SmartDashboard.putNumber("dis to tar", targetDistance);
   


}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerve.drive(new Translation2d(), 0, true, true);
    frc.robot.subsystems.intake.disableFlywheels();
    Constants.flywheelSpeed = 0;
    Constants.wantedShoulderAngle = 1;
    new InstantCommand(() -> new shoulderDown());    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

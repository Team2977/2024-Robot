// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class automaticAiming extends SubsystemBase {

  private final poseEstimator poseSubsystem;
  private final intake intake;
  private final Swerve swerve;
  
  


 
  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(720));
  public final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(0.8, 0, 0, omegConstraints);

  /** Creates a new automaticAiming. */
  public automaticAiming(poseEstimator poseEstimator, intake intake, Swerve swerve) {

  this.poseSubsystem = poseEstimator;
  this.intake = intake;
  this.swerve = swerve;

  pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
    
  pidControllerOmega.setTolerance(Units.degreesToRadians(1));
  pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);
    
    
    if (frc.robot.subsystems.intake.leftInput.get() == true || frc.robot.subsystems.intake.rightInput.get() == true){
      Constants.hasNote = true;
    } else {Constants.hasNote = false;}
   
    
    
    if(targetDistance <= 3.5 &&
        RobotContainer.driverLeftTrigger.getAsBoolean() == false && 
        Constants.autoDriveMode == false &&
        Constants.hasNote == true
        ) {
        Constants.shootBooleanSupplier = () -> true;


    /*     Constants.targetingOn = true;
        new InstantCommand(() -> new indexerIn());
         
        var omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians(), wantedAngle);
        if (pidControllerOmega.atGoal()) {
          omegaSpeed = 0;
        }

     Constants.wantedShoulderAngle = 7.63
                                   + (10.7 * targetDistance) 
                                   - (7.48 * Math.pow(targetDistance, 2)) 
                                   + (1.8 * Math.pow(targetDistance, 3)) 
                                   - (0.148 * Math.pow(targetDistance, 4))
                                   - 0.4;  

    intake.setFlywheelSpeed(96);
  
      SmartDashboard.putNumber("dis to tar", targetDistance);*/


    } else {
      Constants.shootBooleanSupplier = () -> false;
    }
    
    /*else if (Constants.autoDriveMode == false) {
      Constants.targetingOn = false;
      Constants.wantedShoulderAngle = 0;
      intake.disableFlywheels();
      new InstantCommand(() -> new shoulderDown());
      //new InstantCommand(() -> new Swerve().resetOmegaPID());
    }*/


  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.UpperAssembly.indexerIn;
import frc.robot.commands.UpperAssembly.indexerSHOOT;
import frc.robot.commands.UpperAssembly.shoulderDown;


public class automaticAiming extends SubsystemBase {

  private final poseEstimator poseSubsystem;
  private final intake intake;
  private final Swerve swerve;
  private double omegaSpeed;
  private double angleOffset;
  private double shoulderOffset;
  private boolean shootAngleCheck;
  private boolean shootShoulderCheck;
 
  

  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(720));
  public final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);
  public final ProfiledPIDController targetingCheck = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);
  /** Creates a new automaticAiming. */
  public automaticAiming(poseEstimator poseEstimator, intake intake, Swerve swerve) {

  this.poseSubsystem = poseEstimator;
  this.intake = intake;
  this.swerve = swerve;

  pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
  pidControllerOmega.setTolerance(Units.degreesToRadians(3));
  pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);
  shootAngleCheck = false;
  shootShoulderCheck = false;

  targetingCheck.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
  targetingCheck.setTolerance(Units.degreesToRadians(7));
  targetingCheck.enableContinuousInput(Math.PI, -Math.PI);
  }

  /*===========================================================================================*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //vital variables for aiming
    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);
    var wantedAngle = poseSubsystem.getAngleToSpeaker();
  
    //robot aiming angles
      //angles for leading shoots
  if (swerve.getFieldRelativeYVelocity() <= -1 
      && swerve.getFieldRelativeYVelocity() >= -3 
      || swerve.getFieldRelativeYVelocity() >= 1 
      && swerve.getFieldRelativeYVelocity() <= 3) {
            angleOffset = Units.degreesToRadians(swerve.getChassisSpeeds().vyMetersPerSecond * 11);
  } else if (swerve.getFieldRelativeYVelocity() <= -3 || swerve.getFieldRelativeYVelocity() >= 3){
    angleOffset = Units.degreesToRadians(swerve.getChassisSpeeds().vyMetersPerSecond * 10);
  } else {
    angleOffset = 0;
  }

     omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians(), wantedAngle - angleOffset);
        if (pidControllerOmega.atGoal()) {
          omegaSpeed = 0;
        }
     
    targetingCheck.calculate(swerve.getHeading().getRadians(), wantedAngle - angleOffset);

        if (Constants.targetingOn == true && targetDistance <= 4) {
           //send the rotaion value to TeleopSwerve command
  Constants.robotRotationSpeed = omegaSpeed;
        Constants.wantedShoulderAngle = 
          17.6
        - (3.12 * targetDistance)
        - (0.598 * Math.pow(targetDistance, 2))
        + (0.189 * Math.pow(targetDistance, 3))
        + Constants.permanetShoulderOffset
        + Constants.shoulderOffset;
          
        }









/* 
    //check to see if there is a note in the shooter
    if (frc.robot.subsystems.intake.leftInput.get() == false && frc.robot.subsystems.intake.rightInput.get() == false){
      Constants.hasNote = true;
    } else {Constants.hasNote = false;}
   
    
  //checking to see if the robot is in range, the driver is not holding the left trigger, it is not auto mode, and the robot has a note 
    if(targetDistance <= 4 &&
        RobotContainer.driverLeftTrigger.getAsBoolean() == false && 
        Constants.autoDriveMode == false &&
        Constants.hasNote == true
        ) {

  //set targeting on for automatic control of robot rotation        
  Constants.targetingOn = true;

  //bring intake in
  new InstantCommand(() -> new indexerIn());

  //send the rotaion value to TeleopSwerve command
  Constants.robotRotationSpeed = omegaSpeed;

  //set flywheel velocity
  frc.robot.subsystems.intake.setFlywheelSpeed(96);

  //changing shoulder angles from moving around the field
  if(swerve.getFieldRelativeXVelocity() <= -1 || swerve.getFieldRelativeXVelocity() >= 1) {
    shoulderOffset = -swerve.getFieldRelativeXVelocity();
    } else {
        shoulderOffset = 0;
    }

      Constants.wantedShoulderAngle = 
          17.6
        - (3.12 * targetDistance)
        - (0.598 * Math.pow(targetDistance, 2))
        + (0.189 * Math.pow(targetDistance, 3))
        + Constants.permanetShoulderOffset
        + Constants.shoulderOffset;

   


    /*  Constants.wantedShoulderAngle = 7.63
                                   + (10.7 * targetDistance) 
                                   - (7.48 * Math.pow(targetDistance, 2)) 
                                   + (1.8 * Math.pow(targetDistance, 3)) 
                                   - (0.148 * Math.pow(targetDistance, 4))
                                   - 0.5;  */

    
  
       //checks robot angle for auto shooting
   /*  if (poseSubsystem.field2d.getRobotPose().getRotation().getRadians() <= wantedAngle - Units.degreesToRadians(3) 
      || poseSubsystem.field2d.getRobotPose().getRotation().getRadians() >= wantedAngle + Units.degreesToRadians(3)) {shootAngleCheck = true;
      } else {shootAngleCheck = false;}*/
      
      

    
 
 /* 
        //checks shoulder position for auto shooting
   if (Constants.wantedShoulderAngle >= Constants.wantedShoulderAngle - 0.2 
   || Constants.wantedShoulderAngle <= Constants.wantedShoulderAngle + 0.2) {shootShoulderCheck = true; 
      } else {shootShoulderCheck = false;}

       //auto shooting
    if(pidControllerOmega.atGoal() == true && shootShoulderCheck == true && targetDistance <= 3.5) {
      
      frc.robot.subsystems.intake.indexerShoot();
      
    } 
*/
/* 

    //runs if the robot is farther than 4 meters away  
    } else if (Constants.autoDriveMode == false && RobotContainer.driverRightTrigger.getAsBoolean() == false) {
      Constants.targetingOn = false;
      new WaitCommand(0.4);
      Constants.wantedShoulderAngle = -1;
      frc.robot.subsystems.intake.disableFlywheels();
      new InstantCommand(() -> new shoulderDown());
      
    }
  */

    SmartDashboard.putNumber("dis to tar", targetDistance);
    //SmartDashboard.putNumber("omega", omegaSpeed);
    SmartDashboard.putNumber("wanted angle", Units.radiansToDegrees(wantedAngle));
    SmartDashboard.putNumber("wanted shoulder angle", Constants.wantedShoulderAngle);
    SmartDashboard.putNumber("field relitive X", swerve.getFieldRelativeXVelocity());
    SmartDashboard.putNumber("field relitive Y", swerve.getFieldRelativeYVelocity());
    SmartDashboard.putNumber("shoulder offset", Constants.shoulderOffset);
    
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.UpperAssembly.shoulderDown;


public class automaticAiming extends SubsystemBase {

  private final poseEstimator poseSubsystem;
  private final intake intake;
  private final Swerve swerve;
  private double omegaSpeed;
  private double angleOffset;
  private double shoulderOffset;
  //private boolean shootAngleCheck;
  //private boolean shootShoulderCheck;
  //true to make auto shooting not work
  private boolean debuging = false;

 // TreeMap<Double, Double> treeMap = new TreeMap<Double, Double>();
  public static InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(720));
  public final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);
  //public final ProfiledPIDController targetingCheck = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);
  /** Creates a new automaticAiming. */
  public automaticAiming(poseEstimator poseEstimator, intake intake, Swerve swerve) {

  this.poseSubsystem = poseEstimator;
  this.intake = intake;
  this.swerve = swerve;

  pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
  pidControllerOmega.setTolerance(Units.degreesToRadians(3));
  pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);
  //shootAngleCheck = false;
  //shootShoulderCheck = false;


    treeMap.put(1.526, 11.0);
    treeMap.put(1.82, 10.5);
    treeMap.put(2.16, 9.5);
    treeMap.put(2.5, 9.0);
    treeMap.put(2.82, 8.6);
    treeMap.put(3.135, 8.2);
    treeMap.put(3.5, 7.8);
    treeMap.put(3.85, 7.5);
    treeMap.put(4.29, 7.2);

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
    angleOffset = Units.degreesToRadians(swerve.getChassisSpeeds().vyMetersPerSecond * 11);
  } else {
    angleOffset = 0;
  }
      
     omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians(), wantedAngle);
        if (pidControllerOmega.atGoal()) {
          omegaSpeed = 0;
        }
     
   
 
    //check to see if there is a note in the shooter
    if (frc.robot.subsystems.intake.leftInput.get() == false && frc.robot.subsystems.intake.rightInput.get() == false){
      Constants.hasNote = true;
    } else {Constants.hasNote = false;}
   
    
  //checking to see if the robot is in range, the driver is not holding the left trigger, it is not auto mode, and the robot has a note 
    if(targetDistance <= 4 &&
        RobotContainer.driverLeftTrigger.getAsBoolean() == false && 
        Constants.autoDriveMode == false &&
        Constants.hasNote == true &&
        debuging == false
        ) {


  //set targeting on for automatic control of robot rotation        
  Constants.targetingOn = true;

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

    /*   Constants.wantedShoulderAngle = 
          17.6
        - (3.12 * targetDistance)
        - (0.598 * Math.pow(targetDistance, 2))
        + (0.189 * Math.pow(targetDistance, 3))
        + Constants.permanetShoulderOffset
        + Constants.shoulderOffset;*/

        /* 
         Constants.wantedShoulderAngle = 15.4 
                                      - (3.44 * targetDistance)
                                      + (0.357 * Math.pow(targetDistance, 2))
                                      + Constants.permanetShoulderOffset
                                      + Constants.shoulderOffset;*/

    Constants.wantedShoulderAngle = treeMap.get(targetDistance);


    //runs if the robot is farther than 4 meters away  
    } else if (Constants.autoDriveMode == false && RobotContainer.driverRightTrigger.getAsBoolean() == false && debuging == false) {
      Constants.targetingOn = false;
      //new WaitCommand(0.4);
      Constants.wantedShoulderAngle = -1;
      frc.robot.subsystems.intake.disableFlywheels();
      new InstantCommand(() -> new shoulderDown());
    }
    
  

    SmartDashboard.putNumber("dis to tar", targetDistance);
    //SmartDashboard.putNumber("omega", omegaSpeed);
    SmartDashboard.putNumber("wanted angle", Units.radiansToDegrees(wantedAngle));
    SmartDashboard.putNumber("wanted shoulder angle", Constants.wantedShoulderAngle);
    SmartDashboard.putNumber("field relitive X", swerve.getFieldRelativeXVelocity());
    SmartDashboard.putNumber("field relitive Y", swerve.getFieldRelativeYVelocity());
    SmartDashboard.putNumber("shoulder offset", Constants.shoulderOffset);
    
    
  }
}

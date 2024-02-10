// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;

public class aimAndRev extends Command {
  private final intake intake;
  private final Swerve swerve;
  private final PhotonCamera photoncamera;
  private double translationAxis;
  private double strafeAxis;
  private int wantedApriltag;
  private boolean endCommand;
  private List<PhotonTrackedTarget> targets;
  private int targetID;
  
  private final PIDController pidControllerX = new PIDController(0.5, 0.1, 0);
  private final PIDController pidControllerY = new PIDController(0.2, 0.05, 0);
  private final PIDController pidControllerOmega = new PIDController(.4, 0.1, 0);



  public aimAndRev(intake intake, PhotonCamera photonCamera, Swerve swerve) {
    this.intake = intake;
    this.photoncamera = photonCamera;
    this.swerve = swerve;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommand = false;
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset();

   // pidControllerX.setSetpoint(Units.inchesToMeters(100)); // Move forward/backwork to keep 36 inches from the target
    //pidControllerX.setTolerance(Units.inchesToMeters(5));

    //pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    //pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    //pidControllerOmega.setSetpoint(Units.degreesToRadians(-90)); // Rotate the keep perpendicular with the target
    //pidControllerOmega.setTolerance(Units.degreesToRadians(1));

    SmartDashboard.putBoolean("Finding target", true);
    //driver feedback
    RobotContainer.driver.setRumble(RumbleType.kLeftRumble, 0.1);
    RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0.3);


    



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = photoncamera.getLatestResult();
    
      
    
    if (result.hasTargets()) {
    //LIST of targets photon vision has
    targets = result.getTargets();

    //takes the size of the list and uses it to get the 
    int targetListSize = targets.size();

    //checks to see if there is a list of apriltags to check. if no targets are visable, end command
      if (targets.isEmpty()) {endCommand = true;}

    //target ID integer
    targetID = targets.get(targetListSize).getFiducialId();

     var foundTargets = targets.stream().filter(t -> t.getFiducialId() == 4)
                      .filter(t -> !t.equals(4) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                      .findFirst();

      if (foundTargets.isPresent()) {
     var cameraToTarget = foundTargets.get().getBestCameraToTarget();
      

      // X - distance from camera in meters
      // Y - right and left of camera center (in meters?)
      // Z - above and below camera center (in meters?)
      // rotation X - pitch - 0-degrees is flat on floor - rotation is positive as tilted toward camera
      //                    - visible targets in range [0, 180]
      // rotation Y - roll - 0-degrees is straight upward (or straight down) - clockwise rotation is positive
      //                  - seems to give same results if the target is upside down (maybe need to research this one)
      //                  - visible targets are in range [-90, 90]
      // rotation Z - yaw - 0-degrees is perpendicular to the camera, rotated with the right side away from camera (not visible)
      //                  - -90-degrees is straight on with the camera
      //                  - from the camera's perspective, rotation the left side of the target closer is positive
      //                  - visible targets are in range [-180, 0]

      cameraToTarget.getRotation().getAngle();
      SmartDashboard.putNumber("Target X", cameraToTarget.getX());
      SmartDashboard.putNumber("Target Y", Units.metersToInches(cameraToTarget.getY()));
      SmartDashboard.putNumber("Target Z", Units.metersToInches(cameraToTarget.getZ()));
      SmartDashboard.putNumber("Target Rotation X", Units.radiansToDegrees(cameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("Target Rotation Y", Units.radiansToDegrees(cameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("Target Rotation Z", Units.radiansToDegrees(cameraToTarget.getRotation().getZ()));
      SmartDashboard.putNumber("target ID", targetID);
      SmartDashboard.putNumber("target ID 2", foundTargets.get().getFiducialId());




      // Handle rotation using target Yaw/Z rotation
      var targetPitch = cameraToTarget.getRotation().getX();
      double omegaAngle = targetPitch;



       // frc.robot.subsystems.intake.shoulder.set(ControlModeValue.);
        frc.robot.subsystems.intake.shooter.set(Constants.speakerSpeed);
        frc.robot.subsystems.intake.shooterSlave.set(Constants.speakerSpeed);

    }
  }

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(null, 0, true, true);
    frc.robot.subsystems.intake.shooter.set(0);
    frc.robot.subsystems.intake.shooterSlave.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}

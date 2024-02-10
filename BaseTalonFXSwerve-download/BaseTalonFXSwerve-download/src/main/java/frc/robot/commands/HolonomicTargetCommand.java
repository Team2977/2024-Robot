package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;


public class HolonomicTargetCommand extends Command {
  
  private final Swerve swerve;
  private final PhotonCamera photoncamera;
  private final Supplier<Pose2d> poseProvider;
  private double translationAxis;
  private double strafeAxis;
  private int wantedApriltag;
  private boolean endCommand;
  private List<PhotonTrackedTarget> targets;
  private int targetID;
  

  private final PIDController pidControllerX = new PIDController(0.5, 0.1, 0);
  private final PIDController pidControllerY = new PIDController(0.2, 0.05, 0);
  private final PIDController pidControllerOmega = new PIDController(.4, 0.1, 0);

  public HolonomicTargetCommand(Swerve swerve, PhotonCamera photoncamera, Supplier<Pose2d> poseProvider/*  , double translationAxis, double strafeAxis*/) {
    this.swerve = swerve;
    this.photoncamera = photoncamera;
    this.poseProvider = poseProvider;
   // this.translationAxis = translationAxis;
    //this.strafeAxis = strafeAxis;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    endCommand = false;
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset();

    pidControllerX.setSetpoint(Units.inchesToMeters(100)); // Move forward/backwork to keep 36 inches from the target
    pidControllerX.setTolerance(Units.inchesToMeters(5));

    pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(175)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(2));

    SmartDashboard.putBoolean("targeting on", true);
  }

  @Override
  public void execute() {
    var result = photoncamera.getLatestResult();
    
      
    
    if (result.hasTargets()) {
    //LIST of targets photon vision has
    targets = result.getTargets();

    //takes the size of the list and uses it to get the 
    //int targetListSize = targets.size();

    //checks to see if there is a list of apriltags to check. if no targets are visable, end command
      if (targets.isEmpty()) {endCommand = true;}

    //target ID integer
    //targetID = targets.get(targetListSize).getFiducialId();

     var foundTargets = targets.stream().filter(t -> t.getFiducialId() == 4)
                      .filter(t -> !t.equals(4) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                      .findFirst();

      if (foundTargets.isPresent()) {
     var cameraToTarget = foundTargets.get().getBestCameraToTarget();

        
      

      //targetListSize starts at 1. targets.get(int) starts at 0.
    //var cameraToTarget = foundTargets.get().getBestCameraToTarget();
    
      

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
      SmartDashboard.putNumber("found target ID", foundTargets.get().getFiducialId());
      

      
      // Handle distance to target
      var distanceFromTarget = cameraToTarget.getX();
      var xSpeed = pidControllerX.calculate(distanceFromTarget);
      if (pidControllerX.atSetpoint()) {
        xSpeed = 0;
      }

      // Handle alignment side-to-side
      var targetY = cameraToTarget.getY();
      var ySpeed = pidControllerY.calculate(targetY);
      if (pidControllerY.atSetpoint()) {
        ySpeed = 0;
      }

      // Handle rotation using target Yaw/Z rotation
      var targetYaw = cameraToTarget.getRotation().getZ();
      var omegaSpeed = pidControllerOmega.calculate(targetYaw);
      if (pidControllerOmega.atSetpoint()) {
        omegaSpeed = 0;
      }
      swerve.drive(new Translation2d(xSpeed, ySpeed), -omegaSpeed, false, true);
    //  swerve.drive(new Translation2d(translationAxis, strafeAxis).times(Constants.Swerve.maxSpeed), omegaSpeed, false, true);
    //  swerve.drive(new Translation2d(-RobotContainer.driver.getRawAxis(1), -RobotContainer.driver.getRawAxis(0)), 0, true, false);
      
    }  else {
      swerve.drive(new Translation2d(), 0, true, true);
    }
    //if (pidControllerOmega.atSetpoint()) {endCommand = true;}
    if (pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerOmega.atSetpoint()) {
      endCommand = true;
    }
    

  } 


}

  

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("targeting on", false);
    swerve.drive(new Translation2d(), 0, true, false);
  }

  @Override
    public boolean isFinished() {
      //return super.isFinished();
      return endCommand;
    }
}
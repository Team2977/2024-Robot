// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.io.IOException;


import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class poseEstimator extends SubsystemBase {
  /** Creates a new poseEstimator. */

  private final PhotonCamera photonCamera;
  private final Swerve swerve;
  public AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonEstimator;
  private boolean driverStationSet;

  

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> localMesurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(10));

  public final SwerveDrivePoseEstimator poseEstimator;

  public final Field2d field2d = new Field2d();
  

  private double previousPipelineTimestamp = 0;
  

  public poseEstimator(PhotonCamera photonCamera, Swerve swerve) {
    this.photonCamera = photonCamera;
    this.swerve = swerve;
    AprilTagFieldLayout layout;
    driverStationSet = false;

  
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      //layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      var alliance = DriverStation.getAlliance();
      var d = DriverStation.isDSAttached();
     //layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
     // layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
     // layout.setOrigin(alliance.get() == Alliance.Blue ?
       //   OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
 
      if (alliance.get() != Alliance.Red) {
        layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        Constants.onRedTeam = true;
      } else {
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        Constants.onRedTeam = false;
      }

          
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        swerve.getGyroYaw(),
        swerve.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        localMesurementStdDevs);

       
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*  AprilTagFieldLayout layout;

      if(driverStationSet == false && DriverStation.isDSAttached() == true){
       try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance.get() == Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
          
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;
        driverStationSet = true;
      }*/

    // Update pose estimator with the best visible target
      var pipelineResult = photonCamera.getLatestResult();
      var resultTimestamp = pipelineResult.getTimestampSeconds();
      if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();
        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
        if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
          var targetPose = tagPose.get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
  
          var visionMeasurement = camPose.transformBy(Constants.Vision.kRobotToCam);
          poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
          
  }

}
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      swerve.getGyroYaw(),
      swerve.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
    SmartDashboard.putData("pose", field2d);
    SmartDashboard.putNumber("distance to target", getTargetDistance(4));  
    SmartDashboard.putNumber("target yaw radians", getTargetYaw(4));


  



  this.swerve.addVisionMeasurement(poseEstimator.getEstimatedPosition(), resultTimestamp);



    
}

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
      pose.getX(), 
      pose.getY(),
      pose.getRotation().getDegrees());
      }

   public Pose2d getCurrentPose() {
      return poseEstimator.getEstimatedPosition();
      }

  public double getTargetYaw(int wantedTagID){
      var targetYaw = PhotonUtils.getYawToPose(getCurrentPose(), aprilTagFieldLayout.getTagPose(wantedTagID).get().toPose2d()).getRadians();
        return targetYaw;
      }

  public double getTargetDistance(int wantedTagID) {
        var targetDistance = PhotonUtils.getDistanceToPose(getCurrentPose(), aprilTagFieldLayout.getTagPose(wantedTagID).get().toPose2d());
        return targetDistance;
      }

  public Pose2d getTargetPose2d(int wantedApriltag) {
    return aprilTagFieldLayout.getTagPose(wantedApriltag).get().toPose2d();
  }
  

        /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      swerve.getGyroYaw(),
      swerve.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }


}

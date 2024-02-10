// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class twoDTarget extends SubsystemBase {
  /** Creates a new twoDTarget. */
  private PhotonCamera photonCamera;
  private double previousPipelineTimestamp = 0;
  private double yaw;

  public twoDTarget(PhotonCamera photonCamera) {
    this.photonCamera = photonCamera;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();


    
        yaw = target.getYaw();
      
    }
      


  }


  public double getyaw(){
    return yaw;
  }
}

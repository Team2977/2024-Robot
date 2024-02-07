// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.util.concurrent.PriorityBlockingQueue;

import org.ejml.equation.Variable;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class turnToTarget extends Command {
  /** Creates a new turnToTarget. */
  private Swerve swerve;
  private PhotonCamera photonCamera;
  private double rotationValue;
  private PhotonTrackedTarget lastTarget;
  private double yaw;
  

  private final PIDController forwardController = new PIDController(0.1, 0, 0);
  private final PIDController turnController = new PIDController(0.1, 0, 0);




  public turnToTarget(Swerve swerve, PhotonCamera photonCamera) {
    this.swerve = swerve;
    this.photonCamera = photonCamera;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;

    // PID constants should be tuned per robot
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var result = photonCamera.getLatestResult();


    if (result.hasTargets()) {
    
      
     var targetOpt = result.getTargets().stream()
      .filter(t -> t.getFiducialId() == 4)
      .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
      .findFirst();
        
        if (targetOpt.isPresent()) {
          var target = targetOpt.get();
        double yaw = 
          // This is new target data, so recalculate the goal
        if ()
        }
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
      this.swerve.drive(new Translation2d(0,0), rotationValue, true, true);
        

    } else {
        // If we have no targets, stay still.
        this.swerve.drive(new Translation2d(), 0, true, true);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.swerve.drive(new Translation2d(), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

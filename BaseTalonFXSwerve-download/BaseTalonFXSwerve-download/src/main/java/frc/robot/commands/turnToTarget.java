// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import org.photonvision.PhotonCamera;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.twoDTarget;


public class turnToTarget extends Command {
  /** Creates a new turnToTarget. */
  private twoDTarget dTarget;
  private Swerve swerve;
  private PhotonCamera photonCamera;
  private double rotationValue;
  //private PhotonTrackedTarget lastTarget;
  private boolean endCommand;


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
    endCommand = false;
    
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (photonCamera.hasTargets()) {
    if (dTarget.getyaw() <= -5 /*&& dTarget.getyaw() <= 0*/) {
      this.swerve.drive(new Translation2d(), 0.3, true, true);
    } if  (dTarget.getyaw() >= 5) {
      this.swerve.drive(new Translation2d(), -0.3, true, true);
    } }else {
      endCommand = true;
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
    return endCommand;
  }
}

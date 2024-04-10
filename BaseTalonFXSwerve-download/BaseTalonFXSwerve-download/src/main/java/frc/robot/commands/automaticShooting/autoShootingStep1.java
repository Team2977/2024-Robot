// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automaticShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.poseEstimator;

public class autoShootingStep1 extends Command {
  private poseEstimator poseSubsystem;
  /** Creates a new autoShootingStep1. */
  public autoShootingStep1(poseEstimator poseEstimator) {
    this.poseSubsystem = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.AUTOMATIC_AIMING);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.targetingOn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);
    if(targetDistance <= 4){
      Constants.targetingOn = true;
    } else {
      Constants.targetingOn = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.targetingOn;
  }
}

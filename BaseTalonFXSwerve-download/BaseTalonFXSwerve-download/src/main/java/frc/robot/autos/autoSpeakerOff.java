// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class autoSpeakerOff extends Command {
  /** Creates a new autoSpeakerOff. */
  public autoSpeakerOff() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.INTAKE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Constants.wantedShoulderAngle = 0;
      //Constants.speakerSpeed = 0;
      //intake.shooter.setControl(intake.vDC.withVelocity(0));
      //intake.shooterSlave.setControl(intake.vDC.withVelocity(0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.wantedShoulderAngle = 0;
      //Constants.speakerSpeed = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

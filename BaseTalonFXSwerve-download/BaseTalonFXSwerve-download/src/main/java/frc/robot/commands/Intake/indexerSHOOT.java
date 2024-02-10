// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake;

public class indexerSHOOT extends Command {
  /** Creates a new indexerSHOOT. */
  public indexerSHOOT() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {intake.indexer.set(-1);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {intake.indexer.set(-1);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {intake.indexer.set(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

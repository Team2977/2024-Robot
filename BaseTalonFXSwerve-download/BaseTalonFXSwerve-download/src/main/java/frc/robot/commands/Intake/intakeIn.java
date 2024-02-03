// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class intakeIn extends Command {
  /** Creates a new intakeIn. */
  public intakeIn() {addRequirements(RobotContainer.INTAKE);}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.intakeInActive == true) {
      intake.rightIntake.set(Constants.intakeInSpeed);
      intake.leftIntake.set(Constants.intakeInSpeed);
      intake.indexer.set(Constants.indexerIntake);
    } else {
      intake.rightIntake.set(0);
      intake.leftIntake.set(0);
      intake.indexer.set(0);
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {Constants.intakeInActive = !Constants.intakeInActive;}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class shoulderDown extends Command {
  /** Creates a new shoulderDown. */
  private boolean endCommand;
  public shoulderDown() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.INTAKE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {endCommand = false;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    if (intake.shoulder.getPosition().getValueAsDouble() == 0) {
      Constants.wantedShoulderAngle = 0;
      endCommand = true;
    } else {Constants.wantedShoulderAngle = -2;}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.wantedShoulderAngle = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}

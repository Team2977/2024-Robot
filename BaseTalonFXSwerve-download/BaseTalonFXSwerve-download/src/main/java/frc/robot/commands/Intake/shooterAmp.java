// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class shooterAmp extends Command {
  
  /** Creates a new shooterAmp. */
  public shooterAmp() {
    addRequirements(RobotContainer.INTAKE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {intake.shooter.set(Constants.ampSpeed);}

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) {/*intake.indexer.set(0);*/}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

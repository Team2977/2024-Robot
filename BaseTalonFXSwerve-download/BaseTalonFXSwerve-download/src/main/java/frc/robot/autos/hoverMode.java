// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class hoverMode extends Command {
  private intake intake;
  /** Creates a new hoverMode. */
  public hoverMode() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.INTAKE;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    frc.robot.subsystems.intake.rightIntake.set(1);
    frc.robot.subsystems.intake.leftIntake.set(1);
    frc.robot.subsystems.intake.indexer.set(Constants.indexerOutSpeed);
    frc.robot.subsystems.intake.shooter.set(0.15);
    frc.robot.subsystems.intake.shooterSlave.set(-0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

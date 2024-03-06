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
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.rightIntake.set(1);
    intake.leftIntake.set(1);
    
    intake.indexer.set(Constants.indexerOutSpeed);
    intake.shooter.set(0.15);
    intake.shooterSlave.set(-0.05);
    //intake.shooter.set(-1);
    //intake.shooterSlave.set(1);
    //intake.indexer.set(-0.5);
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

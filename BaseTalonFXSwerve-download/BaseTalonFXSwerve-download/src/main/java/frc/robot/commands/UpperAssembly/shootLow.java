// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class shootLow extends Command {
  /** Creates a new shootLow. */
  public shootLow() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.INTAKE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.shooter.setControl(intake.vDC.withVelocity(5));
    //intake.shooterSlave.setControl(intake.vDC.withVelocity(5));
    intake.shooter.set(0.3);
    intake.shooterSlave.set(0.3);
    
    intake.indexer.set(Constants.indexerOutSpeed);
    Constants.flywheelSpeed = 5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.shooter.setControl(intake.vDC.withVelocity(0));
    //intake.shooterSlave.setControl(intake.vDC.withVelocity(0));
    intake.shooter.set(0);
    intake.shooterSlave.set(0);
    intake.indexer.set(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

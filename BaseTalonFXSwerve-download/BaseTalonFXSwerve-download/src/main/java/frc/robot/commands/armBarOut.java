// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class armBarOut extends Command {
  private intake intake;
  /** Creates a new armBarOut. */
  public armBarOut() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //for bumping notes in, 35
    frc.robot.subsystems.intake.ampBar.getPIDController().setReference(48, ControlType.kPosition);
    //Constants.wantedAmpAngle = 45;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.driverPOVLeft.getAsBoolean();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class toggleSpeed extends Command {
  /** Creates a new toggleSpeed. */
  public toggleSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (Constants.fastmode == false) {
      Constants.driveSpeed = 3;
      Constants.turnSpeed = 3;
    } else {
      Constants.driveSpeed = 1;
      Constants.turnSpeed = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { SmartDashboard.putBoolean("fastmode", Constants.fastmode);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.fastmode = !Constants.fastmode;
    SmartDashboard.putBoolean("fastmode", Constants.fastmode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class autoBackUp extends Command {
  private Swerve swerve;
  private boolean endCommand;
  /** Creates a new autoBackUp. */
  public autoBackUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = RobotContainer.s_Swerve;
    addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = RobotContainer.photonCamera.getLatestResult();
    if (result.hasTargets()) {
      swerve.drive(new Translation2d(), 0, true, true);
      endCommand = true;
    } else {
      swerve.drive(new Translation2d(-2, 0), 0, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}

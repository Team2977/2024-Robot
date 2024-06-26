// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class changeAlliance extends Command {
  
  /** Creates a new changeAlliance. */
  public changeAlliance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Constants.onRedTeam = !Constants.onRedTeam;
/* 

Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
        //<RED ACTION>
        Constants.wantedApriltag = 4;
    }
    if (ally.get() == Alliance.Blue) {
      //  <BLUE ACTION>
      Constants.wantedApriltag = 7;
    }
}
else {
    //<NO COLOR YET ACTION>
}
*/

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

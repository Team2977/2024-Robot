// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automaticShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.automaticAiming;

public class automaticIndexerSHOOT extends Command {
  private automaticAiming autoAim;
  private boolean shootShoulderCheck;
  /** Creates a new automaticIndexerSHOOT. */
  public automaticIndexerSHOOT() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootShoulderCheck = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     //checks shoulder position for auto shooting
   if (Constants.wantedShoulderAngle >= Constants.wantedShoulderAngle - 0.2 
   || Constants.wantedShoulderAngle <= Constants.wantedShoulderAngle + 0.2) {shootShoulderCheck = true; 
      } else {shootShoulderCheck = false;}

   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {Constants.indexerShootSpeed = 0;}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

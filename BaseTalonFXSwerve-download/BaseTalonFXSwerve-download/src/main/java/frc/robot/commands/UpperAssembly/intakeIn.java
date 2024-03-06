// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class intakeIn extends Command {
  /** Creates a new intakeIn. */
  public intakeIn() {
    addRequirements(RobotContainer.INTAKE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
      intake.rightIntake.set(1);
      intake.leftIntake.set(1);
      //intake.indexer.set(-0.2);

     
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.intakeInActive = !Constants.intakeInActive;

    intake.rightIntake.set(0);
      intake.leftIntake.set(0);
      intake.indexer.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;





import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake;

public class shooterSpeaker extends Command {
  /** Creates a new shooterSpeaker. */
  public shooterSpeaker() {addRequirements(RobotContainer.INTAKE);}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   var wantedShoulderAngle =  SmartDashboard.getNumber("wanted shoulder angle", 0);
    Constants.wantedShoulderAngle = wantedShoulderAngle;
    intake.shooterSlave.set(0.9);
    intake.shooter.set(0.9);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.shooter.set(0);
    intake.shooterSlave.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
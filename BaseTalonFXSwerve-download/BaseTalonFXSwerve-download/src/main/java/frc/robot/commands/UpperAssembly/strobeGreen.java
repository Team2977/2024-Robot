// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSub;
import frc.robot.subsystems.CANdleSub.AnimationTypes;

public class strobeGreen extends Command {
  private CANdleSub caNdleSub;
  /** Creates a new strobeGreen. */
  public strobeGreen(CANdleSub caNdleSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.caNdleSub = caNdleSub;
    addRequirements(caNdleSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    caNdleSub.changeAnimation(AnimationTypes.greenStrobe);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    caNdleSub.changeAnimation(AnimationTypes.Rainbow);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

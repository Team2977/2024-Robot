// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake;

public class moveShoulder extends Command {
  private final intake intake;
  public static PIDController shoulderPID = new PIDController(0.01, 0, 0);


  /** Creates a new moveShoulder. */
  public moveShoulder(intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* 
    shoulderPID.setSetpoint(-30);
    shoulderPID.setTolerance(2);*/
    SmartDashboard.putBoolean("shoulermoving", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /*   var currentAngle = frc.robot.subsystems.intake.shoulderEncoder.getDistance();
    var shoulderSpeed = shoulderPID.calculate(currentAngle);
    if (shoulderPID.atSetpoint()){
    //Constants.shoulderSpeed = 0;
    }
   */
    Constants.wantedShoulderAngle = 10.5;
    intake.shooter.set(0.9);
    intake.shooterSlave.set(0.9);
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("shoulermoving", false);
    Constants.wantedShoulderAngle = 0;
    intake.shooter.set(0);
    intake.shooterSlave.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

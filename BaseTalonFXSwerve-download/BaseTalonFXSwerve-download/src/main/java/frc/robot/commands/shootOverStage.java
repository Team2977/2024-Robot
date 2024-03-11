// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.poseEstimator;

public class shootOverStage extends Command {
  private final intake intake;
  private final poseEstimator poseSubsystem;
  private final frc.robot.subsystems.Swerve swerve;
  /** Creates a new shootOverStage. */
  public shootOverStage(intake INTAKE, poseEstimator poseEstimator, frc.robot.subsystems.Swerve swerve) {
    this.intake = INTAKE;
    this.poseSubsystem = poseEstimator;
    this.swerve = swerve;
    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.omegaSpeed();
    Constants.wantedShoulderAngle = 10;
    intake.setFlywheelSpeed(96);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableFlywheels();
    swerve.resetOmegaPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

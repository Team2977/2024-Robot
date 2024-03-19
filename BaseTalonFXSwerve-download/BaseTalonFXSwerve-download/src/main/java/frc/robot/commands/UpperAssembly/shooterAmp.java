// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.poseEstimator;

public class shooterAmp extends Command {
  private final Swerve swerve;
  private final intake intake;
  private final poseEstimator poseSubsystem;

  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(500));
  private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);

  
  /** Creates a new shooterAmp. */
  public shooterAmp(Swerve swerve, intake intake, poseEstimator poseEstimator) {
    this.intake = intake;
    this.swerve = swerve;
    this.poseSubsystem = poseEstimator;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //shoulder 8.8 speed 0.8
    Constants.wantedShoulderAngle = 8.8;
    frc.robot.subsystems.intake.setFlywheelSpeed(85);

    var wantedAngle = Units.degreesToRadians(90);
    var omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians(), wantedAngle);
    if (pidControllerOmega.atGoal()) {
      omegaSpeed = 0;
    }
    swerve.drive(
                  new Translation2d(Robot.xSpeed, Robot.ySpeed).times(Constants.Swerve.maxSpeed), 
                  (omegaSpeed / Constants.turnSpeed) * Constants.Swerve.maxAngularVelocity, 
                  true, 
                  true
                  );

  }

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) {
  swerve.drive(new Translation2d(), 0, true, true);
  Constants.wantedShoulderAngle = 1;
  new InstantCommand(() -> new shoulderDown());
  frc.robot.subsystems.intake.disableFlywheels();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

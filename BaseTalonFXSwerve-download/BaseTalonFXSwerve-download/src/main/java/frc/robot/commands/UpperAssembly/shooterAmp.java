// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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


  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 4);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 4);
  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(500));
  private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(Constants.rotaKP, Constants.rotaKI, Constants.rotaKD, omegConstraints);
  private final ProfiledPIDController xController = new ProfiledPIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD, Y_CONSTRAINTS);
  private final Transform3d tagToPose = new Transform3d(new Translation3d(-0.3, 0, 0), new Rotation3d());



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
    
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
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
    var targetPose = poseSubsystem.aprilTagFieldLayout.getTagPose(Constants.wantedAmpTag).get().transformBy(tagToPose);
    var xSpeed = xController.calculate(poseSubsystem.getCurrentPose().getX(), 
                                        targetPose.getX());

    var ySpeed = yController.calculate(poseSubsystem.getCurrentPose().getY(), 
                                        targetPose.getY());


    swerve.drive(
                  new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed), 
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.UpperAssembly.indexerIn;
import frc.robot.commands.UpperAssembly.shoulderDown;


public class automaticAiming extends SubsystemBase {

  private final poseEstimator poseSubsystem;
  private final intake intake;
  private final Swerve swerve;
  public static double rotationValue;

  //private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.feetToMeters(8), Units.feetToMeters(8));
  private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(500));
  public final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(0.1, 0, 0, omegConstraints);

  /** Creates a new automaticAiming. */
  public automaticAiming(poseEstimator poseEstimator, intake intake, Swerve swerve) {

  this.poseSubsystem = poseEstimator;
  this.intake = intake;
  this.swerve = swerve;

  pidControllerOmega.reset(poseSubsystem.field2d.getRobotPose().getRotation().getRadians());
    
  pidControllerOmega.setTolerance(Units.degreesToRadians(1));
  pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    var targetDistance = poseSubsystem.getTargetDistance(Constants.wantedApriltag);
    var wantedAngle = poseSubsystem.getAngleToSpeaker();
    
   
    if(targetDistance <= 4 && RobotContainer.gamepad2.getRawButton(2) == false) {
    Constants.targetingOn = true;
    new InstantCommand(() -> new indexerIn());


      var omegaSpeed = pidControllerOmega.calculate(swerve.getHeading().getRadians() - Units.degreesToRadians(1), wantedAngle);
    if (pidControllerOmega.atGoal()) {
      omegaSpeed = 0;
    }
      rotationValue = (omegaSpeed / Constants.turnSpeed);
    /*swerve.drive(
                  new Translation2d(Robot.xSpeed, Robot.ySpeed).times(Constants.Swerve.maxSpeed), 
                  (omegaSpeed / Constants.turnSpeed) * Constants.Swerve.maxAngularVelocity, 
                  true, 
                  true
                  );*/

    Constants.wantedShoulderAngle = 7.63
                                   + (10.7 * targetDistance) 
                                   - (7.48 * Math.pow(targetDistance, 2)) 
                                   + (1.8 * Math.pow(targetDistance, 3)) 
                                   - (0.148 * Math.pow(targetDistance, 4))
                                   - 0.4;  

    frc.robot.subsystems.intake.shooter.setControl(intake.vDC.withVelocity(96));
    frc.robot.subsystems.intake.shooterSlave.setControl(intake.vDC.withVelocity(96));

      SmartDashboard.putNumber("dis to tar", targetDistance);


    } else {
      Constants.targetingOn = false;
      //swerve.drive(new Translation2d(), 0, true, true);
      Constants.wantedShoulderAngle = -1;
      intake.shooter.setControl(intake.vDC.withVelocity(0));
      intake.shooterSlave.setControl(intake.vDC.withVelocity(0));
      intake.shooter.set(0);
      intake.shooterSlave.set(0);
      new InstantCommand(() -> new shoulderDown());
    
    }


  }
}

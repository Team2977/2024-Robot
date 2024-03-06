// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.UpperAssembly;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.turnToTarget;
import frc.robot.subsystems.poseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shoot extends SequentialCommandGroup {
  /** Creates a new shoot. */
  private poseEstimator poseEstimator;
  private Supplier<Pose2d> poseProvider;
  public shoot(Supplier<Pose2d> poseProvider) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.poseProvider = poseProvider;
    addCommands(
      new turnToTarget(RobotContainer.s_Swerve, RobotContainer.photonCamera, poseProvider, RobotContainer.poseESTIMATOR)
      
          );
  }
}

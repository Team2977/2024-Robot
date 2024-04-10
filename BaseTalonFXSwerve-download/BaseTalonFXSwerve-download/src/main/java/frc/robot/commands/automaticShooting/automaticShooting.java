// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automaticShooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.poseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class automaticShooting extends SequentialCommandGroup {
  private poseEstimator poseSubsystem;
  /** Creates a new automaticShooting. */
  public automaticShooting(poseEstimator poseEstimator) {
    this.poseSubsystem = poseEstimator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autoShootingStep1(poseSubsystem),
      new automaticIndexerSHOOT().withTimeout(5),
      new automaticShootingStep2()
    );
  }
}

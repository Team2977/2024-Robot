// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpperAssembly.indexerSHOOT;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoshoot extends SequentialCommandGroup {
  /** Creates a new autoshoot. */
  public autoshoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autoSpeakerOn(RobotContainer.INTAKE, RobotContainer.s_Swerve, RobotContainer.poseESTIMATOR).withTimeout(1.5),
      new indexerSHOOT().withTimeout(0.5),
      new autoSpeakerOff().withTimeout(0.1),
      new autoShoulderDown().withTimeout(0.1)
    );
  }
}

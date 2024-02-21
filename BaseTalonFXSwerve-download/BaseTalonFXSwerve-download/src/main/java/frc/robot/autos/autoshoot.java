// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.UpperAssembly.indexerSHOOT;
import frc.robot.commands.UpperAssembly.shoulderDown;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoshoot extends SequentialCommandGroup {
  /** Creates a new autoshoot. */
  public autoshoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autoSpeakerOn().withTimeout(3),
      new indexerSHOOT().withTimeout(1),
      new autoSpeakerOff().withTimeout(0.5),
      new shoulderDown().withTimeout(0.5)
    );
  }
}

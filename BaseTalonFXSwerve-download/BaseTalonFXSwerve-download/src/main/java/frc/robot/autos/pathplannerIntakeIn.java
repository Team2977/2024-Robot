package frc.robot.autos;


import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class pathplannerIntakeIn extends SequentialCommandGroup {
private Swerve swerve;

    public pathplannerIntakeIn(Swerve s_Swerve){
        this.swerve = s_Swerve;
        addCommands(
             new autoIntakeIn().withTimeout(2.5)
        );
        
    }
}
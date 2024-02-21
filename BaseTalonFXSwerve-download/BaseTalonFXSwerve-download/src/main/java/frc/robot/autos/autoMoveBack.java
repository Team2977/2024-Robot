package frc.robot.autos;


import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class autoMoveBack extends SequentialCommandGroup {
private Swerve swerve;

    public autoMoveBack(Swerve s_Swerve){
        this.swerve = s_Swerve;
        addCommands(
             new autoBackUp()
             
            
        );
        
    }
}
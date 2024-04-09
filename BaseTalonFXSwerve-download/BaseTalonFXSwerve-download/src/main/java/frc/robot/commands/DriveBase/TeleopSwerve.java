package frc.robot.commands.DriveBase;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private static double translationVal;
    private static double strafeVal;
    private static double rotationVal;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
         
         //strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
         //translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
         //rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

         
         if (Constants.targetingOn == true){
            rotationVal = Constants.robotRotationSpeed;
                if(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) < 0) {
                    translationVal = -0.5;
                } else if (MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) > 0) {
                    translationVal = 0.5;
                } else {
                    translationVal = 0;
                }
                
                if (MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) < 0){
                    strafeVal = -0.5;
                } else if (MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) > 0) {
                    strafeVal = 0.5;
                } else {
                    strafeVal = 0;
                }

        } else {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
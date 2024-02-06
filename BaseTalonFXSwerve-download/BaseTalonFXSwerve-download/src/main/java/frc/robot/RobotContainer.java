package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.DriveBase.TeleopSwerve;
import frc.robot.commands.DriveBase.toggleSpeed;
import frc.robot.commands.Intake.intakeIn;
import frc.robot.commands.Intake.intakeOut;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    public final static Joystick gamepad2 =  new Joystick(1);
    if (isReal()) {
        // Instantiate IO implementations to talk to real hardware
        driveTrain = new DriveTrain(DriveTrainIOReal());
        elevator = new Elevator(ElevatorIOReal());
        intake = new Intake(IntakeIOReal());
    } else {
        // Use anonymous classes to create "dummy" IO implementations
        driveTrain = new DriveTrain(DriveTrainIO() {});
        elevator = new Elevator(ElevatorIO() {});
        intake = new Intake(IntakeIO() {});
    }
    

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  /*   private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 3;
*/

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverX = new JoystickButton(driver, 3);
    private final JoystickButton rightBummber = new JoystickButton(driver, 6);
    private final JoystickButton driverIntakeIn = new JoystickButton(driver, 2);
    private final JoystickButton driverIntakeOut = new JoystickButton(driver, 1);


   /*  private final JoystickButton rightBummber = new JoystickButton(driver, 8);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    private final JoystickButton robotCentric = new JoystickButton(driver, 7);
    private final JoystickButton driverX = new JoystickButton(driver, 5);
*/


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public static final intake INTAKE = new intake();


        SendableChooser<Command> chooser = new SendableChooser<>();
        private final Command autointakeCommand = new autoIntakeIn();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (Constants.autoDriveMode == false) {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis)/Constants.driveSpeed, 
                () -> driver.getRawAxis(strafeAxis)/Constants.driveSpeed, 
                () -> driver.getRawAxis(rotationAxis)/Constants.turnSpeed, 
                () -> robotCentric.getAsBoolean()
            )
        );
            } else {
                s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> Constants.autoTranslation/Constants.driveSpeed, 
                () -> Constants.autoStrafe/Constants.driveSpeed, 
                () -> Constants.autoRotation/Constants.turnSpeed, 
                () -> robotCentric.getAsBoolean()
            )
        );
            }


        

        
        chooser.addOption("autointake", autointakeCommand);






        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driverX.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
       
        rightBummber.onTrue(new toggleSpeed());
        driverIntakeIn.onTrue(new intakeIn());
        driverIntakeOut.onTrue(new intakeOut());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}

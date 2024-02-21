package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.DriveBase.TeleopSwerve;
import frc.robot.commands.DriveBase.toggleSpeed;
import frc.robot.commands.UpperAssembly.indexerIn;
import frc.robot.commands.UpperAssembly.indexerSHOOT;
import frc.robot.commands.UpperAssembly.intakeIn;
import frc.robot.commands.UpperAssembly.intakeOut;
import frc.robot.commands.UpperAssembly.moveShoulder;
import frc.robot.commands.UpperAssembly.shooterSpeaker;
import frc.robot.commands.UpperAssembly.shoulderDown;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public static final Joystick driver = new Joystick(0);
    public static final Joystick gamepad2 =  new Joystick(1);
    public static final PhotonCamera photonCamera = new PhotonCamera("frontCamera");
    public final SendableChooser<Command> chooser;
    

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 3;


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);  //Y
    private final JoystickButton robotCentric = new JoystickButton(driver, 7);
    private final JoystickButton driverY = new JoystickButton(driver, 5); //reset modules to absolute
    private final JoystickButton rightBummber = new JoystickButton(driver, 8); 
    private final JoystickButton driverIntakeIn = new JoystickButton(driver, 1); //A button
    private final JoystickButton driverIntakeOut = new JoystickButton(driver, 2); //B button
    private final JoystickButton driverRightPaddle = new JoystickButton(driver, 3);//aim at speaker
    private final JoystickButton driverLeftPaddle =  new JoystickButton(driver, 6);
    private final JoystickButton driverLeftTrigger = new JoystickButton(driver, 9);
    private final JoystickButton driverRightTrigger = new JoystickButton(driver, 10);
    private final JoystickButton driverStart = new JoystickButton(driver, 12);

    private final JoystickButton GA = new JoystickButton(gamepad2, 1);
    private final JoystickButton GLeftBumper = new JoystickButton(gamepad2, 5);
 
    private final JoystickButton GX = new JoystickButton(gamepad2, 3);
 /*   private final JoystickButton GLB = new JoystickButton(gamepad2, XboxController.Button.kLeftBumper.value);
    private final JoystickButton GA = new JoystickButton(gamepad2, 1);
    private final JoystickButton GB = new JoystickButton(gamepad2, 2);
 */   
   /*  private final JoystickButton rightBummber = new JoystickButton(driver, 8);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    private final JoystickButton robotCentric = new JoystickButton(driver, 7);
    private final JoystickButton driverX = new JoystickButton(driver, 5);
*/


    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final intake INTAKE = new intake();
    public static final poseEstimator poseESTIMATOR = new poseEstimator(photonCamera, s_Swerve);
    public static final chaseTag CHASETAG = new chaseTag(photonCamera, s_Swerve, poseESTIMATOR::getCurrentPose);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis)/Constants.driveSpeed * Constants.invert, 
                () -> driver.getRawAxis(strafeAxis)/Constants.driveSpeed * Constants.invert, 
                () -> driver.getRawAxis(rotationAxis)/Constants.turnSpeed * Constants.invert, 
                () -> robotCentric.getAsBoolean()
            )
        );
           

            NamedCommands.registerCommand("autoSpeakerOn", new autoSpeakerOn());
            NamedCommands.registerCommand("intakeIn", new intakeIn());
            NamedCommands.registerCommand("intakeOut", new intakeOut());
            NamedCommands.registerCommand("autoShoot", new autoshoot());
            NamedCommands.registerCommand("indexerSHOOT", new indexerSHOOT());
            NamedCommands.registerCommand("autoShoot", new autoshoot());
            NamedCommands.registerCommand("setRobotPose", new setRobotPose());
            NamedCommands.registerCommand("autoMoveBack", new autoMoveBack(s_Swerve));
            NamedCommands.registerCommand("autoBackUp", new autoBackUp());
            NamedCommands.registerCommand("setRobotPose", new setRobotPose());
            NamedCommands.registerCommand("autoIntakeIn", new autoIntakeIn());
            NamedCommands.registerCommand("hoverMode", new hoverMode());
            NamedCommands.registerCommand("aimAndRev", new aimAndRev(INTAKE, photonCamera, s_Swerve, poseESTIMATOR::getCurrentPose, poseESTIMATOR));


        
        chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        chooser.addOption("move", new PathPlannerAuto("testAuto"));
        chooser.addOption("test", new exampleAuto(s_Swerve));
        chooser.addOption("blue test auto", new PathPlannerAuto("bluetestauto"));
        chooser.addOption("hover mode", new PathPlannerAuto("hover mode"));
        chooser.addOption("blue 3 note", new PathPlannerAuto("Blue 3 note"));
       // chooser.addOption("blue 3 note", new PathPlannerAuto("Blue 3 note auto"));
        chooser.addOption("Blue 3 note v2", new PathPlannerAuto("Blue 3 note v2"));
        chooser.addOption("shoot 1 don't move", new PathPlannerAuto("1 note auto"));
        chooser.addOption("move and shoot 1", new PathPlannerAuto("move and shoot 1"));
        SmartDashboard.putData("Auto Mode", chooser);






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
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driverY.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        zeroGyro.onTrue(new InstantCommand(() -> poseESTIMATOR.setCurrentPose(new Pose2d())));
    
        rightBummber.onTrue(new toggleSpeed());
        driverIntakeIn.onTrue(new intakeIn());
        driverIntakeOut.onTrue(new intakeOut());
        driverLeftTrigger.whileTrue(new moveShoulder(INTAKE));
        driverLeftTrigger.onFalse(new shoulderDown());
        driverRightTrigger.whileTrue(new indexerSHOOT());

        driverLeftTrigger.whileTrue(new aimAndRev(INTAKE, photonCamera, s_Swerve, poseESTIMATOR::getCurrentPose, poseESTIMATOR));
        driverRightPaddle.whileTrue(new indexerIn());
        driverStart.onTrue(new changeAlliance());

       // driverRightPaddle.whileTrue(new HolonomicTargetCommand(s_Swerve, photonCamera, poseESTIMATOR::getCurrentPose));
       // driverLeftPaddle.whileTrue(new HolonomicTargetCommand(s_Swerve, photonCamera, poseESTIMATOR::getCurrentPose));
       // driverRightPaddle.whileTrue(new turnToTarget(s_Swerve, photonCamera, poseESTIMATOR::getCurrentPose, poseESTIMATOR));

       // driverAim.whileTrue(new aimAndRev(INTAKE, photonCamera, s_Swerve));
       // driverLeftPaddle.whileTrue(new shoot(poseESTIMATOR::getCurrentPose));
      
      GA.whileTrue(new moveShoulder(INTAKE));
      GLeftBumper.whileTrue(new indexerIn());
      GX.whileTrue(new shooterSpeaker());
       /* 
        GX.whileTrue(new shooterSpeaker());
        GLB.whileTrue(new indexerSHOOT());
        GB.whileTrue(new chaseTag(photonCamera, s_Swerve, poseESTIMATOR::getCurrentPose));
        GA.whileTrue(new moveShoulder(INTAKE));*/
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return chooser.getSelected();
    }
}

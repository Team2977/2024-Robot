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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.DriveBase.TeleopSwerve;
import frc.robot.commands.DriveBase.toggleSpeed;
import frc.robot.commands.UpperAssembly.indexerIn;
import frc.robot.commands.UpperAssembly.indexerSHOOT;
import frc.robot.commands.UpperAssembly.intakeIn;
import frc.robot.commands.UpperAssembly.intakeOut;
import frc.robot.commands.UpperAssembly.shootLow;
import frc.robot.commands.UpperAssembly.shooterAmp;
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
    public static final PhotonCamera backCamera = new PhotonCamera("backCamera");
    
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
    public static final JoystickButton driverLeftTrigger = new JoystickButton(driver, 9);
    private final JoystickButton driverRightTrigger = new JoystickButton(driver, 10);
    private final JoystickButton driverSelect = new JoystickButton(driver, 11);
    private final JoystickButton driverStart = new JoystickButton(driver, 12);


    private final JoystickButton GA = new JoystickButton(gamepad2, 1);
    private final JoystickButton GB = new JoystickButton(gamepad2, 2);
    private final JoystickButton GX = new JoystickButton(gamepad2, 3);
    private final JoystickButton GY = new JoystickButton(gamepad2, 4);
    private final JoystickButton GLeftBumper = new JoystickButton(gamepad2, 5);
    private final JoystickButton GRightBumper = new JoystickButton(gamepad2, 6);
    

    //trigers
    

    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    public static final intake INTAKE = new intake();
    public static final poseEstimator poseESTIMATOR = new poseEstimator(photonCamera, backCamera, s_Swerve);
    public static final automaticAiming AUTOMATIC_AIMING = new automaticAiming(poseESTIMATOR, INTAKE, s_Swerve);
    public static final chaseTag CHASETAG = new chaseTag(photonCamera, s_Swerve, poseESTIMATOR::getCurrentPose);
    



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis)/Constants.driveSpeed * Constants.invert, 
                () -> driver.getRawAxis(strafeAxis)/Constants.driveSpeed * Constants.invert, 
                () -> -driver.getRawAxis(rotationAxis)/Constants.turnSpeed /* Constants.invert*/, 
                () -> robotCentric.getAsBoolean()
            )
        );
           
            
            NamedCommands.registerCommand("autoSpeakerOn", new autoSpeakerOn(INTAKE, photonCamera, s_Swerve, poseESTIMATOR));
            NamedCommands.registerCommand("intakeIn", new intakeIn());
            NamedCommands.registerCommand("intakeOut", new intakeOut());
            NamedCommands.registerCommand("autoShoot", new autoshoot());
            NamedCommands.registerCommand("indexerSHOOT", new indexerSHOOT());
            NamedCommands.registerCommand("autoShoot", new autoshoot());
            NamedCommands.registerCommand("autoINTAKEv2", new pathplannerIntakeIn(s_Swerve));
            NamedCommands.registerCommand("autoBackUp", new autoBackUp());
            NamedCommands.registerCommand("autoIntakeIn", new autoIntakeIn());
            NamedCommands.registerCommand("hoverMode", new hoverMode());
            NamedCommands.registerCommand("aimAndRev", new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
            NamedCommands.registerCommand("null", null);


        
        chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        //3 note autos
        chooser.addOption("3 note amp side", new PathPlannerAuto("3 note amp side"));
        chooser.addOption("3 note source side", new PathPlannerAuto("3 note source side"));
        chooser.addOption("3 note source far", new PathPlannerAuto("3 note source far"));
        //2 note autos
        chooser.addOption("2 note amp", new PathPlannerAuto("2 note amp"));
        chooser.addOption("2 note podium", new PathPlannerAuto("2 note podium"));
        chooser.addOption("shoot and move 2", new PathPlannerAuto("shoot and move 2"));
        //shoot only
        chooser.addOption("1 note auto", new PathPlannerAuto("1 note auto"));
        //shoot and move 1
        chooser.addOption("shoot and move 1", new PathPlannerAuto("shoot and move 1"));
        //4 note auto
        chooser.addOption("4 note amp side", new PathPlannerAuto("4 note amp side"));
        chooser.addOption("4 note source side", new PathPlannerAuto("4 note source side"));
        //Hover modes
        chooser.addOption("hover mode", new PathPlannerAuto("hover mode"));
        

        
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
        driverY.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        zeroGyro.onTrue(new InstantCommand(() -> poseESTIMATOR.setCurrentPose(new Pose2d())));
        //driverStart.onTrue(new changeAlliance());
        rightBummber.onTrue(new toggleSpeed());

        //intake control
        driverIntakeIn.onTrue(new intakeIn());
        driverIntakeOut.onTrue(new intakeOut());
        driverRightPaddle.whileTrue(new indexerIn());

        //aim, and after relese, put shoulder down
        //driverLeftTrigger.whileTrue(new aimAndRev(INTAKE, photonCamera, s_Swerve, poseESTIMATOR));
        driverLeftTrigger.whileTrue(new shooterAmp(s_Swerve, INTAKE, poseESTIMATOR));
        driverLeftTrigger.onFalse(new shoulderDown());
        driverLeftPaddle.onTrue(new shoulderDown());

        driverStart.whileTrue(new shootOverStage(INTAKE, poseESTIMATOR, s_Swerve));
        driverStart.onFalse(new shoulderDown());
        driverSelect.whileTrue(new shootLow());

        //shoot
        driverRightTrigger.whileTrue(new indexerSHOOT());
        //AUTOMATIC_AIMING.shootTrigger.whileTrue(new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
        //AUTOMATIC_AIMING.shootTrigger.onFalse(new shoulderDown());
       /*  INTAKE.shootTrigger.whileFalse(new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
        INTAKE.shootTriggerLeft.whileFalse(new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
        INTAKE.shootTrigger.onTrue(new shoulderDown()); 
        INTAKE.shootTriggerLeft.onTrue(new shoulderDown());
        */
      
        //Operator controls
        GA.whileTrue(new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
        GA.onFalse(new shoulderDown());
        GLeftBumper.whileTrue(new indexerIn());

        
        GY.whileTrue(new shootLow());
        GX.whileTrue(new shooterSpeaker());
        GX.onFalse(new shoulderDown());
       
        GRightBumper.whileTrue(new shooterAmp(s_Swerve, INTAKE, poseESTIMATOR));
        GRightBumper.onFalse(new shoulderDown());
        

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

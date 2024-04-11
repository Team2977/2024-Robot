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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.DriveBase.TeleopSwerve;
import frc.robot.commands.DriveBase.toggleSpeed;
import frc.robot.commands.UpperAssembly.indexerIn;
import frc.robot.commands.UpperAssembly.indexerSHOOT;
import frc.robot.commands.UpperAssembly.intakeIn;
import frc.robot.commands.UpperAssembly.intakeOut;
import frc.robot.commands.UpperAssembly.moveShoulder;
import frc.robot.commands.UpperAssembly.shootLow;
import frc.robot.commands.UpperAssembly.shooterAmp;
import frc.robot.commands.UpperAssembly.shooterSpeaker;
import frc.robot.commands.UpperAssembly.shoulderDown;
import frc.robot.commands.shooterTrim.shooterTrimDown;
import frc.robot.commands.shooterTrim.shooterTrimUp;
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
    public static final PhotonCamera photonCamera = new PhotonCamera("frontCamera1");
    public static final PhotonCamera backCamera = new PhotonCamera("backCamera");
    
    public final SendableChooser<Command> chooser;
    

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 3;


    /* Driver Buttons */
   /*  private final JoystickButton driverIntakeIn = new JoystickButton(driver, 1); //A button
    private final JoystickButton driverIntakeOut = new JoystickButton(driver, 2); //B button
    private final JoystickButton driverRightPaddle = new JoystickButton(driver, 3);//aim at speaker
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);  //Y
    private final JoystickButton driverY = new JoystickButton(driver, 5); //reset modules to absolute
    private final JoystickButton driverLeftPaddle =  new JoystickButton(driver, 6);
    private final JoystickButton robotCentric = new JoystickButton(driver, 7);
    private final JoystickButton rightBummber = new JoystickButton(driver, 8);    
    private final JoystickButton driverLeftTrigger = new JoystickButton(driver, 9);
    private final JoystickButton driverRightTrigger = new JoystickButton(driver, 10);
    private final JoystickButton driverStart = new JoystickButton(driver, 12);

    private final JoystickButton GA = new JoystickButton(gamepad2, 1);
    private final JoystickButton GB = new JoystickButton(gamepad2, 2);
    private final JoystickButton GX = new JoystickButton(gamepad2, 3);
    private final JoystickButton GY = new JoystickButton(gamepad2, 4);
    private final JoystickButton GLeftBumper = new JoystickButton(gamepad2, 5);
    private final JoystickButton GRightBumper = new JoystickButton(gamepad2, 6);*/
 
 /* Driver Buttons */
    private final JoystickButton driverIntakeIn = new JoystickButton(driver, 1); //A button. intake in
    private final JoystickButton driverIntakeOut = new JoystickButton(driver, 2); //B button. intake out
    private final JoystickButton driverRightPaddle = new JoystickButton(driver, 3);//right paddle. indexer in
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);  //X button. zero gyro
    private final JoystickButton driverY = new JoystickButton(driver, 5); //Y button. reset modules to aboslute
    private final JoystickButton driverLeftPaddle =  new JoystickButton(driver, 6);// left paddle. shoulderDown
    private final JoystickButton robotCentric = new JoystickButton(driver, 7); //left bumper. robot centric
    private final JoystickButton rightBummber = new JoystickButton(driver, 8); //right bumper. change speeds 
    public static final JoystickButton driverLeftTrigger = new JoystickButton(driver, 9); //left trigger. amp 
    public static final JoystickButton driverRightTrigger = new JoystickButton(driver, 10); //right trigger. indexer shoot
    private final JoystickButton driverSelect = new JoystickButton(driver, 11); //select. shoot low.
    private final JoystickButton driverStart = new JoystickButton(driver, 12); //start. shoot over stage
    private final POVButton driverPOVUp = new POVButton(driver, 0); //dpad up
    private final POVButton driverPOVDown = new POVButton(driver, 180); //dpad down
    public static final POVButton driverPOVLeft = new POVButton(driver, 270); //dpad left
    public static final POVButton driverPOVRight = new POVButton(driver, 90); //dpad right
    

    private final JoystickButton GA = new JoystickButton(gamepad2, 1); //GA
    private final JoystickButton GB = new JoystickButton(gamepad2, 2);//GB
    private final JoystickButton GX = new JoystickButton(gamepad2, 3);//GX
    private final JoystickButton GY = new JoystickButton(gamepad2, 4); //GY
    private final JoystickButton GLeftBumper = new JoystickButton(gamepad2, 5); //GLeftBumper
    private final JoystickButton GRightBumper = new JoystickButton(gamepad2, 6); //GRightBumper

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final intake INTAKE = new intake();
    public static final poseEstimator poseESTIMATOR = new poseEstimator(photonCamera, backCamera, s_Swerve);
    public static final automaticAiming AUTOMATIC_AIMING = new automaticAiming(poseESTIMATOR, INTAKE, s_Swerve);
    public static final chaseTag CHASETAG = new chaseTag(photonCamera, s_Swerve, poseESTIMATOR::getCurrentPose);
    public static final CANdleSub candleSUB = new CANdleSub();

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
            NamedCommands.registerCommand("setRobotPose", new setRobotPose());
            NamedCommands.registerCommand("autoINTAKEv2", new pathplannerIntakeIn(s_Swerve));
            NamedCommands.registerCommand("autoBackUp", new autoBackUp());
            NamedCommands.registerCommand("setRobotPose", new setRobotPose());
            NamedCommands.registerCommand("autoIntakeIn", new autoIntakeIn());
            NamedCommands.registerCommand("hoverMode", new hoverMode());
            NamedCommands.registerCommand("aimAndRev", new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));


        
        chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        /* 
        chooser.addOption("move", new PathPlannerAuto("testAuto"));
        //chooser.addOption("test", new exampleAuto(s_Swerve));
        chooser.addOption("blue test auto", new PathPlannerAuto("bluetestauto"));
        chooser.addOption("hover mode", new PathPlannerAuto("hover mode"));
        chooser.addOption("blue 3 note", new PathPlannerAuto("Blue 3 note"));
       // chooser.addOption("blue 3 note", new PathPlannerAuto("Blue 3 note auto"));
        chooser.addOption("Blue 3 note v2", new PathPlannerAuto("Blue 3 note v2"));
        chooser.addOption("shoot 1 don't move", new PathPlannerAuto("1 note auto"));
        */

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
        //driverY.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        zeroGyro.onTrue(new InstantCommand(() -> poseESTIMATOR.setCurrentPose(new Pose2d())));
        rightBummber.onTrue(new toggleSpeed());

        //intake control
        driverIntakeIn.onTrue(new intakeIn());
        driverIntakeOut.onTrue(new intakeOut());
        driverRightPaddle.whileTrue(new indexerIn());
        //driverRightPaddle.onTrue(new shoulderDown());

        //aim, and after relese, put shoulder down
        driverLeftTrigger.whileTrue(new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
        //driverLeftTrigger.whileTrue(new shooterAmp(s_Swerve, INTAKE, poseESTIMATOR));
        //driverLeftTrigger.onFalse(new armBarIn());
        driverLeftTrigger.onFalse(new shoulderDown());
        
        //driverY.whileTrue(new shooterAmp(s_Swerve, INTAKE, poseESTIMATOR));
        //driverY.onFalse(new shoulderDown());
        //driverLeftPaddle.onTrue(new shoulderDown());
        driverLeftPaddle.whileTrue(new shooterAmp(s_Swerve, INTAKE, poseESTIMATOR));
        driverLeftPaddle.onFalse(new shoulderDown());

        //extra stuff
        driverStart.whileTrue(new shootOverStage(INTAKE, poseESTIMATOR, s_Swerve));
        driverStart.onFalse(new shoulderDown());
        driverSelect.whileTrue(new shootLow());
        driverSelect.onFalse(new shoulderDown());

        //shooter trim
        driverPOVUp.onTrue(new shooterTrimUp());
        driverPOVDown.onTrue(new shooterTrimDown());

        //shoot
        driverRightTrigger.whileTrue(new indexerSHOOT());
        //shootTrigger.whileTrue(new indexerSHOOT());
       //driverPOVRight.whileTrue(new armBarIn());
       //driverPOVLeft.whileTrue(new armBarOut());
      
        /*Operator controls*/
        GY.whileTrue(new aimAndRev(INTAKE, s_Swerve, poseESTIMATOR));
        GY.onFalse(new shoulderDown());
        GLeftBumper.whileTrue(new indexerIn());

        //extra controls
        //GY.whileTrue(new shootLow());
        GA.whileTrue(new shooterSpeaker());
        GA.onFalse(new shoulderDown());

       //amp shoot
        GX.whileTrue(new shooterAmp(s_Swerve, INTAKE, poseESTIMATOR));
        GX.onFalse(new shoulderDown());

        //G4.whileTrue(new strobeGreen(candleSub));
        
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

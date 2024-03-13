package frc.robot;



import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static  double driveSpeed;
    public static  double turnSpeed;
    public static boolean slowMode = true;
    public static double wantedShoulderAngle;
    public static double wantedClimberPose;
    public static double shoulderSpeed;
    public static int wantedApriltag = 7;
    public static boolean onRedTeam;
    public static int invert;


    //shooting stuff
    public static double flywheelSpeed;
    public static boolean shoot;
    public static boolean hasNote;
    public static boolean targetingOn;
    public static double robotRotationSpeed;
    public static BooleanSupplier shootBooleanSupplier;

    //rotation PID values
    public static final double rotaKP = 0.8;
    public static final double rotaKI = 0;
    public static final double rotaKD = 0;
    

    //auto mode constants
    public static boolean autoDriveMode = false;
    public static double autoTranslation = 0;
    public static double autoStrafe = 0;
    public static double autoRotation = 0;

    public static boolean intakeInActive = false;
    public static boolean intakeOutActive = false;
    //not auto mode constants
    public static double speakerSpeed = 96;
    public static final double ampSpeed = 30;
    public static double indexerShootSpeed;
    public static final double indexerIntake = 0.5;
    public static final double intakeInSpeed = -0.1;
    public static final double indexerOutSpeed = 0.9;
    public static final double climberMaxPose = 10;

/*===============================================================================================================================*/
public static class Vision {
        public static final String kCameraName = "photonvision";
        // Cam mounted facing forward, 17 inches forward of center, 11 inches left of center, 8 inches up from ground, and rotated 20 degrees upward 
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(-17), Units.inchesToMeters(-11), Units.inchesToMeters(8)), new Rotation3d(0, Units.degreesToRadians(-20), 0));
        
                // Cam mouned facing backwards, 17 inches back of center, 11 inches right of center, 5 inches up from groud, rotated 20 degrees up, and rotated 180 degrees to face backwards
        public static final Transform3d robotToBackCam = 
                new Transform3d(new Translation3d(Units.inchesToMeters(17), Units.inchesToMeters(11), Units.inchesToMeters(8)), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
        
                // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //public static final AprilTagFieldLayout kTagLayout =
          //      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
/*===============================================================================================================================*/
    

    public static final class Swerve {
        public static final int pigeonID = 61;

        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26.6); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //+,+
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //+,-
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //-,+
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); //-,-

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 40;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.5; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
      
        /*  public static final double driveKS = (0.055763 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (0.00042305 / 12);
        public static final double driveKA = (0.00001722 / 12);*/
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.feetToMeters(13); //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = Units.degreesToRadians(500); //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Right Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int canCoderID = 42;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-130);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Right Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(298);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(158);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

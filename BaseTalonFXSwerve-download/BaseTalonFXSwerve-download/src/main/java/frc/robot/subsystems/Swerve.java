package frc.robot.subsystems;

import frc.robot.SwerveModule;


import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator swerveOdometry;
    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    private poseEstimator poseEstimator;
   
    private final TrapezoidProfile.Constraints omegConstraints = new Constraints(Units.degreesToRadians(500), Units.degreesToRadians(720));
    public final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(0.8, 0, 0, omegConstraints);


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "driveBase");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        this.poseEstimator = RobotContainer.poseESTIMATOR;
        
        pidControllerOmega.setTolerance(Units.degreesToRadians(1));
        pidControllerOmega.enableContinuousInput(Math.PI, -Math.PI);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

         
        // Define the standard deviations for the pose estimator, which determine how fast the pose
        // estimate converges to the vision measurement. This should depend on the vision measurement
        // noise
        // and how many or how frequently vision measurements are applied to the pose estimator.
         var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
         
          swerveOdometry =
          new SwerveDrivePoseEstimator(
                  Constants.Swerve.swerveKinematics,
                  getGyroYaw(),
                  getModulePositions(),
                  new Pose2d(),
                  stateStdDevs,
                  visionStdDevs);
  
         // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::autoResetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    edu.wpi.first.math.util.Units.inchesToMeters(25), // Drive base radius in meters. Distance from robot center to furthest module. 
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    
  }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    




    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

//this is the original translation2d[] method. if stuff doesn't work, go back to this 
/*public Translation2d[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    } */

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
       
    }

    public void setPose(Pose2d pose) {
        
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
        
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
 
    public void runVelocity(ChassisSpeeds speeds) {
        this.setChassisSpeeds(speeds, true, true);
    }    


    public void autoResetPose(Pose2d pose2d) {
    //does nothing, but can't remove. required for pathplaner.
    }

    /*========================================================================================================= */

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        swerveOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
        setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds));
        this.targetChassisSpeeds = targetChassisSpeeds;
    }

    /** Get the chassis speeds of the robot (vx, vy, omega) from the swerve module states. */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return new ChassisSpeeds(
                getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos()
                        - getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
                getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getCos()
                        + getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
                getChassisSpeeds().omegaRadiansPerSecond);
    }

    public double getFieldRelativeXVelocity() {
        return getFieldRelativeChassisSpeeds().vxMetersPerSecond;
    }

    public double getFieldRelativeYVelocity() {
        return getFieldRelativeChassisSpeeds().vyMetersPerSecond;
    }

    public double getFieldRelativeAngularVelocity() {
        return getFieldRelativeChassisSpeeds().omegaRadiansPerSecond;
    }


    @Override
    public void periodic(){
       
       
        swerveOdometry.update(getGyroYaw(), getModulePositions());
 
      /*  for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }  */       
       
        SmartDashboard.putNumber("Vel X", this.getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("vel Y", this.getChassisSpeeds().vyMetersPerSecond);
        
      
    }


    
}
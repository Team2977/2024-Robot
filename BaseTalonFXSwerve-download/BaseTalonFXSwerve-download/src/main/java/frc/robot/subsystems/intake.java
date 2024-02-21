// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class intake extends SubsystemBase {
  /** Creates a new intake. */

  //intakes
  public static final CANSparkMax rightIntake = new CANSparkMax(1, MotorType.kBrushless);
  public static final CANSparkMax leftIntake = new CANSparkMax(2, MotorType.kBrushless);
  //omni wheels on the top of the shooter
  public static final CANSparkMax indexer = new CANSparkMax(52, MotorType.kBrushless);
  //shooting flywheel
  public static final TalonFX shooter = new TalonFX(5);
  public static final TalonFX shooterSlave = new TalonFX(4);
  //motor that moves the shooter up and down. also the encoder for that motor
  public static final TalonFX shoulder = new TalonFX(51);
  //climber
  public static final TalonFX winch = new TalonFX(6);
  //shooter lock
  public static final CANSparkMax lock = new CANSparkMax(0, MotorType.kBrushless);


  //private MotionMagicDutyCycle mmDC = new MotionMagicDutyCycle(0);
  private PositionDutyCycle mmDC = new PositionDutyCycle(0);

  public intake() {
    shooterSlave.setInverted(true);

    
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 1; // 1 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 50; // Take approximately 0.2 seconds to reach max accel 
    
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 0.03; //0.05
    slot0.kI = 0.01;
    slot0.kD = 0;
    slot0.kV = 5;
    slot0.kS = 0.5; // Approximately 0.5V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fdb.RotorToSensorRatio = 1;
    
    HardwareLimitSwitchConfigs hls = cfg.HardwareLimitSwitch;
    hls.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hls.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    hls.ReverseLimitEnable = true;
    hls.ReverseLimitAutosetPositionValue = 0;
    hls.ReverseLimitAutosetPositionEnable = true;
 
    
    
    shoulder.getConfigurator().apply(cfg);

    rightIntake.setSmartCurrentLimit(40);
    rightIntake.setIdleMode(IdleMode.kCoast);
    leftIntake.setSmartCurrentLimit(40);
    leftIntake.setIdleMode(IdleMode.kCoast);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // shoulder.set(RobotContainer.gamepad2.getRawAxis(5)/5);
    //indexer.set(RobotContainer.gamepad2.getRawAxis(1));
   // shooter.set(RobotContainer.gamepad2.getRawAxis(4));
    //rightIntake.set(RobotContainer.gamepad2.getRawAxis(0));
    //leftIntake.set(-RobotContainer.gamepad2.getRawAxis(0));    
    winch.set(RobotContainer.gamepad2.getRawAxis(1));
    indexer.set(Constants.indexerShootSpeed);
    shoulder.setControl(mmDC.withPosition(Constants.wantedShoulderAngle));
     
     SmartDashboard.putNumber("shoulder Pos", shoulder.getPosition().getValueAsDouble());
     SmartDashboard.putString("strijg", shoulder.getPosition().getUnits());
     


    
  }

 

}

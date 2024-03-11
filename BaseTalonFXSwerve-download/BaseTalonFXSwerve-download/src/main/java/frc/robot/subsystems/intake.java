// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  //motor that moves the shooter up and down
  public static final TalonFX shoulder = new TalonFX(51);
  //climber
  public static final TalonFX leftHook = new TalonFX(6);
  public static final TalonFX rightHook = new TalonFX(7);
  

  private PositionDutyCycle mmDC = new PositionDutyCycle(0);
  public static VelocityDutyCycle vDC = new VelocityDutyCycle(0);
  
  //limit switch that doesn't exist. for testing trigger
  private final DigitalInput digitalInput = new DigitalInput(1);
  @SuppressWarnings("checkstyle:MemberName")
  public final Trigger hasCargo = new Trigger(digitalInput::get);

  public intake() {
    
    rightIntake.setSmartCurrentLimit(100);
    rightIntake.setIdleMode(IdleMode.kCoast);
    rightIntake.setInverted(true);
    leftIntake.setSmartCurrentLimit(100);
    leftIntake.setIdleMode(IdleMode.kCoast);
    indexer.setSmartCurrentLimit(30);
   
    //shoulder configs
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 1; // 1 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 50; // Take approximately 0.2 seconds to reach max accel 
    
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 0.08; //0.05
    slot0.kI = 0;
    slot0.kD = 0.01;
    slot0.kV = 10;
    slot0.kS = 2; // Approximately 0.5V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fdb.RotorToSensorRatio = 1;
    
    HardwareLimitSwitchConfigs hls = cfg.HardwareLimitSwitch;
    hls.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hls.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    hls.ReverseLimitEnable = true;
    hls.ReverseLimitAutosetPositionValue = 0;
    hls.ReverseLimitAutosetPositionEnable = true;
 
    cfg.CurrentLimits.SupplyCurrentLimit = 30;
    cfg.CurrentLimits.SupplyCurrentThreshold = 40;
    cfg.CurrentLimits.SupplyTimeThreshold = 0.1;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    shoulder.getConfigurator().apply(cfg);
    shoulder.setNeutralMode(NeutralModeValue.Brake);

    
    
    //shooter configs
    TalonFXConfiguration configsShooter = new TalonFXConfiguration();
    /* Voltage-based velocity requires  feed forward to account for the back-emf of the motor */
    configsShooter.Slot0.kP = 0.13; //0.11 An error of 1 rotation per second results in 2V output
    configsShooter.Slot0.kI = 0; //0.5 An error of 1 rotation per second increases output by 0.5V every second
    configsShooter.Slot0.kD = 0; //0.0001 A change of 1 rotation per second squared results in 0.01 volts output
    configsShooter.Slot0.kV = 0.01; //0.12 Falcon 500 is  500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
   
    // Peak output of 100 volts
    //configsShooter.Voltage.PeakForwardVoltage = 100;
    //configsShooter.Voltage.PeakReverseVoltage = -100;

    // shooter current limits
    configsShooter.CurrentLimits.SupplyCurrentLimit = 25;
    configsShooter.CurrentLimits.SupplyCurrentThreshold = 40;
    configsShooter.CurrentLimits.SupplyTimeThreshold = 0.1;
    configsShooter.CurrentLimits.SupplyCurrentLimitEnable = true;
    

    shooter.getConfigurator().apply(configsShooter);
    shooterSlave.getConfigurator().apply(configsShooter);


    shooter.setInverted(true);
    shooterSlave.setInverted(false);
    shooter.setNeutralMode(NeutralModeValue.Coast);
    shooterSlave.setNeutralMode(NeutralModeValue.Coast);
    indexer.setInverted(true);

//climber configs
  TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.Slot2.kP = 1;
    climberConfig.Slot2.kI = 0;
    climberConfig.Slot2.kD = 0;
    climberConfig.Slot2.kV = 0;
    climberConfig.Slot2.kS = 1;
    climberConfig.Slot2.kG = 0;

    //climberConfig.Voltage.PeakForwardVoltage = 100;
    //climberConfig.Voltage.PeakReverseVoltage = -100;

    //climber current limits
    climberConfig.CurrentLimits.SupplyCurrentLimit = 30;
    climberConfig.CurrentLimits.SupplyCurrentThreshold =  40;
    climberConfig.CurrentLimits.SupplyTimeThreshold = 1;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    MotionMagicConfigs climberMagicConfigs = climberConfig.MotionMagic;
    climberMagicConfigs.MotionMagicCruiseVelocity = 50; //2 rotations per second
    climberMagicConfigs.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    climberMagicConfigs.MotionMagicJerk = 50; // Take approximately 0.2 seconds to reach max accel 
    
    SoftwareLimitSwitchConfigs climberSoftlimit = climberConfig.SoftwareLimitSwitch;
    climberSoftlimit.ForwardSoftLimitThreshold = 160;
    climberSoftlimit.ForwardSoftLimitEnable = true;
    climberSoftlimit.ReverseSoftLimitThreshold = 0;
    climberSoftlimit.ReverseSoftLimitEnable = true;

    leftHook.getConfigurator().apply(climberConfig, 0.02);
    rightHook.getConfigurator().apply(climberConfig, 0.02);

    leftHook.setNeutralMode(NeutralModeValue.Brake);
    rightHook.setNeutralMode(NeutralModeValue.Brake);
    leftHook.setInverted(false);
    rightHook.setInverted(true);
    
  }

  public static void setFlywheelSpeed(double speed) {
    shooter.setControl(vDC.withVelocity(speed));
    shooterSlave.setControl(vDC.withVelocity(speed));
  }

  public static void disableFlywheels() {
    shooter.setControl(vDC.withVelocity(0));
    shooterSlave.setControl(vDC.withVelocity(0));
    shooter.set(0);
    shooterSlave.set(0);
  }



  @Override
  public void periodic() {    
    indexer.set(Constants.indexerShootSpeed);
    shoulder.setControl(mmDC.withPosition(Constants.wantedShoulderAngle));

    double climberControls = MathUtil.applyDeadband(-RobotContainer.gamepad2.getRawAxis(1), 0.1);
    leftHook.set(climberControls);
    rightHook.set(climberControls);
    

    
     
     // SmartDashboard.putNumber("shoulder Pos", shoulder.getPosition().getValueAsDouble());
      //SmartDashboard.putNumber("left motor", shooterSlave.getVelocity().getValueAsDouble());
     // SmartDashboard.putNumber("right motor", shooter.getVelocity().getValueAsDouble());
    
     
     //runs during auto
     if (Constants.autoDriveMode == true) {
      shooter.setControl(vDC.withVelocity(Constants.speakerSpeed));
      shooterSlave.setControl(vDC.withVelocity(Constants.speakerSpeed));
      //rightIntake.set(1);
      //leftIntake.set(1);
    }
      


  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class intake extends SubsystemBase {
  /** Creates a new intake. */

  //intakes
  public static final CANSparkMax rightIntake = new CANSparkMax(1, MotorType.kBrushless);
  public static final CANSparkMax leftIntake = new CANSparkMax(2, MotorType.kBrushless);
  //omni wheels on the top of the shooter
  public static final CANSparkMax indexer = new CANSparkMax(3, MotorType.kBrushless);
  //shooting flywheel
  public static final CANSparkMax shooter = new CANSparkMax(4, MotorType.kBrushless);
  //motor that moves the shooter up and down
  public static final TalonFX shoulder = new TalonFX(51);
  //climber
  public static final TalonFX winch = new TalonFX(6);



  public intake() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulder.set(RobotContainer.gamepad2.getRawAxis(1)/5);
  }





}

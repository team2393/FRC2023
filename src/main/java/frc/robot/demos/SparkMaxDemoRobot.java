// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.demos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Demo of SparkMAX with mini 550 motor */
public class SparkMaxDemoRobot extends CommandBaseRobot
{
  CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  XboxController controller = new XboxController(0);

  @Override
  public void robotInit()
  {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic()
  {
    motor.setVoltage(controller.getLeftX());
    // Position in turns, speed in RPM
    SmartDashboard.putNumber("Turns", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM", motor.getEncoder().getVelocity());
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.parts.SparkMini;

/** Motor Test robot */
public class MotorTestRobot extends TimedRobot
{
  private final XboxController joystick = new XboxController(0);

  // CTRE now has two APIs:
  // The new "PhoenixPro" com.ctre.phoenixpro.hardware.TalonFX
  // for C:\Users\Public\Documents\FRC\2023 Firmware\PhoenixPro\TalonFX-Application-23.4.0.3-Pro-season2023.crf
  // which complains at runtime with
  // "Device is not licensed. Cannot get any data from it."

  // private final TalonFX motor= new TalonFX(3+1);
  
  // .. and the older "Phoenix 5"
  // com.ctre.phoenix.motorcontrol.can.TalonFX with wrapper WPI_TalonFX
  // that runs with 
  // C:\Users\Public\Documents\FRC\2023 Firmware\Phoenix\TalonFX-Application-22.1.1.0-season2023.crf
  private final WPI_TalonFX motor = new WPI_TalonFX(3+1);

  // Simple PWM controller:
  // private final SparkMini motor = new SparkMini(0);

  @Override
  public void robotInit()
  {
    System.out.println("************************************");
    System.out.println("**  " + getClass().getName());
    System.out.println("************************************");
  }

  @Override
  public void teleopPeriodic()
  {
    // Right stick "forward" to move motor "forward"
    motor.set(-joystick.getRightY());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/** Motor Test robot
 */
public class MotorTestRobot extends TimedRobot
{
  XboxController joystick = new XboxController(0);
  WPI_TalonFX motor = new WPI_TalonFX(1);

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

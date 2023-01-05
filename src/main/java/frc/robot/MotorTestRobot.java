// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.parts.SparkMini;

/** Motor Test robot
 */
public class MotorTestRobot extends TimedRobot
{
  private final XboxController joystick = new XboxController(0);
  private final SparkMini motor = new SparkMini(0);

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

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervebot;

import frc.robot.CommandBaseRobot;
import frc.robot.swervelib.SwerveOI;

public class RotatorTestRobot extends CommandBaseRobot
{
  SparkMiniRotator rotator = new SparkMiniRotator(0, -16);

  @Override
  public void teleopPeriodic()
  {
    rotator.setVoltage(SwerveOI.joystick.getLeftX()*-5);
  }

  @Override
  public void autonomousPeriodic()
  {
    // Cycle between 0 and 90 degrees every 5 seconds
    long cycle = (System.currentTimeMillis() / 5000) % 2;
    double angle = cycle * 90.0;
    rotator.setAngle(angle);
  }
}

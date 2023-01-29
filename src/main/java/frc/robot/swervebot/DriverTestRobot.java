// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervebot;

import frc.robot.CommandBaseRobot;
import frc.robot.swervelib.SwerveOI;

/** Driver test robot */
public class DriverTestRobot extends CommandBaseRobot
{
  FalconDriver driver = new FalconDriver(0);

  @Override
  public void disabledInit()
  {
    driver.brake(false);
  }

  @Override
  public void teleopInit()
  {
    driver.brake(true);
  }

  @Override
  public void teleopPeriodic()
  {
    driver.setVoltage(SwerveOI.joystick.getRightY()*-5);
  }

  @Override
  public void autonomousPeriodic()
  {
    // Cycle between two speeds every 5 seconds
    long cycle = (System.currentTimeMillis() / 5000) % 2;
    double speed = cycle == 0 ? 0.5 : 1.5;
    driver.setSpeed(speed);
  }
}

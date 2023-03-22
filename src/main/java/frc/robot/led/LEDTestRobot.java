// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CommandBaseRobot;

public class LEDTestRobot extends CommandBaseRobot
{
  private LED led = new LED();

  @Override
  public void disabledInit()
  {
    led.getDefaultCommand().schedule();
  }

  @Override
  public void teleopInit()
  {
    new WarbleCommand(led, Color.kPurple).schedule();
    // new WarbleCommand(led, Color.kYellow).schedule();
  }

  @Override
  public void autonomousInit()
  {
    new GreenGoldBlink(led).schedule();
    // new MovingColorList(led, 10, Color.kWhite, Color.kRed, Color.kBlue).schedule();
  }
}

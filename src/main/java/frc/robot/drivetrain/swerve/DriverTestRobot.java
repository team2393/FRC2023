// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Driver Test robot */
public class DriverTestRobot extends CommandBaseRobot
{
  private final XboxController joystick = new XboxController(0);

  private final Driver driver = new Driver(1);

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("Setpoint", 0.0);
  }

  @Override
  public void teleopPeriodic()
  {
    // Stick "Forward" to run motor forward
    driver.run(-joystick.getRightY());
  }

  @Override
  public void autonomousPeriodic()
  {
    driver.setSpeed(SmartDashboard.getNumber("Setpoint", 0.0));
  }
}

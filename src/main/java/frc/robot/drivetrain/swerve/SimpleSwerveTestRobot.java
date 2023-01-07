// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandBaseRobot;

/** Simple Swerve Test robot */
public class SimpleSwerveTestRobot extends CommandBaseRobot
{
  private final Drivetrain drivetrain = new Drivetrain();

  private Command drive = new DriveCommand(drivetrain);
  private Command swerve = new SwerveCommand(drivetrain);
  
  @Override
  public void teleopInit()
  {
    swerve.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    if (OI.joystick.getLeftBumperPressed())
      drive.schedule();
    if (OI.joystick.getRightBumperPressed())
      swerve.schedule();
  }
}

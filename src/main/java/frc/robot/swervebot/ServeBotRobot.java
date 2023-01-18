package frc.robot.swervebot;
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandBaseRobot;

/** ServeBot */
public class ServeBotRobot extends CommandBaseRobot
{
  private final SwervebotDrivetrain drivetrain = new SwervebotDrivetrain();
  private final XboxController joystick = new XboxController(0);

  @Override
  public void teleopPeriodic()
  {
    double vx = -joystick.getRightY();
    double vy = -joystick.getRightX();
    double vr = -joystick.getLeftX();
    drivetrain.swerve(vx, vy, vr, new Translation2d());
  }
}

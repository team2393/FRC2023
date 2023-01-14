// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Rotator Test robot */
public class RotatorTestRobot extends CommandBaseRobot
{
  private final XboxController joystick = new XboxController(0);

  // Module   Offset    Position
  // 0       -18        Front left
  // 1        90        Front right
  // 2      -161        Back right
  // 3      -107        Back left
  private final Rotator rotator = new Rotator(0, 0.0);

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("Setpoint", 0.0);
  }

  @Override
  public void teleopPeriodic()
  {
    // Moving stick to the right rotates
    // clockwise, i.e., towards negative angles
    final double angle = -180.0 * joystick.getRightX();
    rotator.setAngle(angle);
  }

  @Override
  public void autonomousPeriodic()
  {
    rotator.setAngle(SmartDashboard.getNumber("Setpoint", 0.0));
  }
}

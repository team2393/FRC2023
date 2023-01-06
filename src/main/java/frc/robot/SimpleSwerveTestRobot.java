// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drivetrain.swerve.SwerveModule;

/** Simple Swerve Test robot */
public class SimpleSwerveTestRobot extends TimedRobot
{
  private final XboxController joystick = new XboxController(0);

  private final SwerveModule[] modules = new SwerveModule[]
  {
    new SwerveModule(0, SwerveModule.OFFSETS[0]),
    new SwerveModule(1, SwerveModule.OFFSETS[1]),
    new SwerveModule(2, SwerveModule.OFFSETS[2]),
    new SwerveModule(3, SwerveModule.OFFSETS[3])
  };
 
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
    // x = "forward"
    // y = "left"
    final double x = -joystick.getRightY();
    final double y = -joystick.getRightX();

    // Speed: -1 back .. +1 m/s forward
    final double speed = Math.abs(x*x+y*y);

    // Angle: 0 = forward, 90 = left
    final double angle;
    if (speed < 0.1)
      angle = 0;
    else
      angle = Math.toDegrees(Math.atan2(y, x));

    // Run all modules at the same angle and speed
    for (SwerveModule module : modules)
      module.setSwerveModule(angle, speed);
  }
}

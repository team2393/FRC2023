// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for simplistic swerve moves */
public class DriveCommand extends CommandBase
{
  private final Drivetrain drivetrain;

  public DriveCommand(Drivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    // x = "forward"
    // y = "left"
    final double x = -OI.joystick.getRightY();
    final double y = -OI.joystick.getRightX();

    // Speed: -1 back .. +1 m/s forward
    final double speed = Math.abs(x*x+y*y);

    // Angle: 0 = forward, 90 = left
    final double angle;
    if (speed < 0.1)
      angle = 0;
    else
      angle = Math.toDegrees(Math.atan2(y, x));

    // Run all modules at the same angle and speed
    drivetrain.drive(angle, speed);
  }
}

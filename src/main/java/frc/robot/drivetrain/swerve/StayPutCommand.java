// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj2.command.RunCommand;

/** Command for staying put */
public class StayPutCommand extends RunCommand
{
  public StayPutCommand(Drivetrain drivetrain, double angle)
  {
    super(() -> drivetrain.drive(angle, 0),
          drivetrain);
  }
}

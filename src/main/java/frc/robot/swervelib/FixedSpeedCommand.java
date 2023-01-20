// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixedSpeedCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;
  private final double speed;

  public FixedSpeedCommand (SwerveDrivetrain drivetrain, double speed)
  {
    this.drivetrain = drivetrain;
    this.speed = speed;
    addRequirements(drivetrain);
  }

  @Override
  public void execute()
  {
    drivetrain.drive(0, speed);
  }
}

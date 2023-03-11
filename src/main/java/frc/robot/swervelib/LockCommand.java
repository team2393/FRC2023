// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to Lock drivetrain */
public class LockCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;

  public LockCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    drivetrain.lock();;
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for interactive swerve moves */
public class SwerveCommand extends CommandBase
{
  private final Drivetrain drivetrain;

  public SwerveCommand(Drivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    drivetrain.swerve(OI.getForwardSpeed(),
                      OI.getLeftSpeed(),
                      OI.getRotationSpeed(),
                      new Translation2d(0, 0));
  }
}

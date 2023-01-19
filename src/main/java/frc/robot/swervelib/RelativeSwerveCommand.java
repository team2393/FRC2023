// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for human to drive robot in direction in which it's currently pointed
 *
 *  'Forward' means go whereever the nose of the robot points
 */
public class RelativeSwerveCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;
  private final SelectCenter center;

  public RelativeSwerveCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    center = new SelectCenter(drivetrain);
    addRequirements(drivetrain);
  }

  public void execute()
  {
    drivetrain.swerve(SwerveOI.getForwardSpeed(),
                      SwerveOI.getLeftSpeed(),
                      SwerveOI.getRotationSpeed(),
                      center.determineCenter());
  }
}

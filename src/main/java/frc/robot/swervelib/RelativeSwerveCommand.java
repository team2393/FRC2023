// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for human to drive robot in direction in which it's currently pointed
 *
 *  'Forward' means go whereever the nose of the robot points
 */
public class RelativeSwerveCommand extends CommandBase
{
  private static final Translation2d CENTER = new Translation2d(0, 0);
  private final SwerveDrivetrain drivetrain;
  private final SelectCenter center;

  public RelativeSwerveCommand(SwerveDrivetrain drivetrain)
  {
    this(drivetrain, false);
  }

  /** @param drivetrain
   *  @param select_center Support selectable rotation center?
   */
  public RelativeSwerveCommand(SwerveDrivetrain drivetrain, boolean select_center)
  {
    this.drivetrain = drivetrain;
    if (select_center)
      center = new SelectCenter(drivetrain);
    else
      center = null;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    Translation2d axis = center == null
                       ? CENTER
                       : center.determineCenter();
    drivetrain.swerve(SwerveOI.getForwardSpeed(),
                      SwerveOI.getLeftSpeed(),
                      SwerveOI.getRotationSpeed(),
                      axis);
  }
}

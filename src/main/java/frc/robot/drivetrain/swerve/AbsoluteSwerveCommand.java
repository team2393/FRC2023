// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for human to drive robot in absolute field coordinates
 *
 *  'Forward' means 'up' in the field, no matter which way the
 *  nose of the robot points
 */
public class AbsoluteSwerveCommand extends CommandBase
{
  private final static Translation2d CENTER = new Translation2d();
  private final Drivetrain drivetrain;

  public AbsoluteSwerveCommand(Drivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    // Speed vector (vx, vy) meant to be in field coordinates,
    // vx going "up" from the origin of the field along X
    double vx = OI.getForwardSpeed();
    double vy = OI.getLeftSpeed();

    // If robot also points 'up', we could use (vx, vy) as given,
    // but generally we need to rotate (vx, vy) backwards from the current heading
    // of the robot to see how the robot needs to move:
    Rotation2d correction = Rotation2d.fromDegrees(-drivetrain.getHeading());
    Translation2d absoluteDirection = new Translation2d(vx, vy).rotateBy(correction);

    // Swerve robot in 'absoluteDirection', while rotating as requested
    double vr = OI.getRotationSpeed();
    drivetrain.swerve(absoluteDirection.getX(), absoluteDirection.getY(), vr, CENTER);
  }
}

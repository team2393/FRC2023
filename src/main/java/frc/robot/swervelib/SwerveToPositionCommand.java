// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.AutoNoMouse;

/** Command for swerving to a desired location
 *
 *   Created trajectory from whereever we are to
 *   the target location, a straight line.
 *   Schedules a new trajectory following command
 *   for that path and then waits for it to finish.
 */
public class SwerveToPositionCommand extends ProxyCommand
{
  /** @param drivetrain
   *  @param Target Desired pose
   */
  public SwerveToPositionCommand(SwerveDrivetrain drivetrain, Pose2d target)
  {
    this(drivetrain, target.getX(), target.getY(), target.getRotation().getDegrees());
  }

  /** @param drivetrain
   *  @param x Desired X [m],
   *  @param y Y [m],
   *  @param angle Angle [degrees]
   */
  public SwerveToPositionCommand(SwerveDrivetrain drivetrain, double x, double y, double angle)
  {
    super(() ->
    {
      Pose2d current = drivetrain.getPose();
      double dist = Math.hypot(x - current.getX(), y - current.getY());

      double heading = Math.toDegrees(Math.atan2(y - current.getY(),
                                                 x - current.getX()));

      try
      {
            Trajectory path = AutoNoMouse.createTrajectory(true,
                                                          current.getX(), current.getY(), heading,
                                                          x, y, heading);
            // Proxied command does NOT hold drivetrain, we do.
            // If the proxied command held the drivetrain, and this command was in a sequence,
            // then starting the proxied command would stop the sequence and thus this command?!
            return drivetrain.createTrajectoryCommand(path, angle, false);
      }
      catch (Exception ex)
      {
        System.err.println("Cannot create trajectory for distance of " + dist + " meters");
      }

      return new PrintCommand("Trajectory Error");
    });

    // Hold drivetrain
    addRequirements(drivetrain);
  }
}

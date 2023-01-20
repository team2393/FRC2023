// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoNoMouse;

/** Command for swerving to a desired location
 *
 *   Created trajectory from whereever we are to
 *   the target location, a straight line.
 *   Schedules a new trajectory following command
 *   for that path and then waits for it to finish.
 */
public class SwerveToPositionCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;
  private final double target_x, target_y, final_angle;
  private Command follower;
   
  /** @param drivetrain
   *  @param x Desired X [m],
   *  @param y Y [m],
   *  @param angle Angle [degrees]
   */
  public SwerveToPositionCommand(SwerveDrivetrain drivetrain, double x, double y, double angle)
  {
    this.drivetrain = drivetrain;
    target_x = x;
    target_y = y;
    final_angle = angle;
    // Do NOT require the drivebase. The `follower` will do that.
 }

  @Override
  public void initialize()
  {
    Pose2d current = drivetrain.getPose();

    double heading = Math.toDegrees(Math.atan2(target_y - current.getY(),
                                               target_x - current.getX()));
    Trajectory path = AutoNoMouse.createTrajectory(true,
                                                   current.getX(), current.getY(), heading,
                                                   target_x, target_y, heading);
    follower = drivetrain.createTrajectoryCommand(path, final_angle);
    follower.schedule();
  }

  @Override
  public boolean isFinished()
  {
    return follower.isFinished();
  }
}

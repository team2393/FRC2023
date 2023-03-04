// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for autonomously driving uphill */
public class AutoDriveUphillCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;

  /** Start position */
  private Translation2d start;

  /** Angle at which we'll run +-1m/s */
  private double max_speed_angle;

  /** Which way is 'uphill' (heading relative to robot) */
  private double uphill;

  /** How long have we been stable? */
  private Timer stable_timer = new Timer();

  public AutoDriveUphillCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    // Remember where we think we are
    start = drivetrain.getPose().getTranslation();

    // Initially, this angle results in driving +-1 m/s
    max_speed_angle = 25.0;
    // Don't know which way is uphill, yet
    uphill = Double.NaN;
    stable_timer.restart();
  }

  public void execute()
  {
    // Any "nose up" angle means we need to drive forward (X)
    double vx = drivetrain.getPitch()/max_speed_angle;
    // Any "left up" angle means we need to drive left (Y)
    double vy = drivetrain.getRoll()/max_speed_angle;
    // We ain't gonna not do no rotating, maybe
    double vr = 0.0;

    drivetrain.swerve(vx, vy, vr);

    // Are we moving, i.e., not stable?
    if ((vx*vx + vy*vy) > 0.2)
    {
      stable_timer.restart();

      // Did we aleady determine which way is "uphill"?
      if (Double.isNaN(uphill))
      {
        uphill = Math.toDegrees(Math.atan2(vy, vx));
        System.out.println("Uphill is " + uphill + " deg");
      }
      else
      { // Did we change direction from original "uphill" by at least 90 degrees?
        double now = Math.toDegrees(Math.atan2(vy, vx));
        if (Math.abs(Math.IEEEremainder(uphill-now, 360)) > 90)
        {
          uphill = now;
          max_speed_angle += 3.5;
          System.out.println("Uphill changed to " + now + " deg, max speed angle now " + max_speed_angle);
        }
      } 
    }
  }

  @Override
  public boolean isFinished()
  {
    // Start position may not be the correct field location,
    // but should be good enough to check how far we have moved since then.
    double distance = drivetrain.getPose().getTranslation().getDistance(start);
    // If we moved more than 2 meters, assume something's wrong
    if (distance > 2.0)
    {
      System.out.println("Aborting AutoDriveUphillCommand, moved " + distance + " meters");
      return true;
    }
    // Have we been standing still for a while, likely leveled out?
    if (stable_timer.hasElapsed(10.0))
    {
      System.out.println("AutoDriveUphillCommand believes that we did it!");
      return true;
    }
    return false;
  }
}

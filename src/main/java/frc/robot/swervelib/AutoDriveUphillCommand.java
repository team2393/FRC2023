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
  }

  public void execute()
  {
    // Angles are reported in degrees.
    // If we're tilted 45 degrees we want to run at +-1 m/s uphill.

    // Any "nose up" angle means we need to drive forward (X)
    double vx = drivetrain.getPitch()/45.0;
    // Any "left up" angle means we need to drive left (Y)
    double vy = drivetrain.getRoll()/45.0;
    // We ain't gonna not do no rotating, maybe
    double vr = 0.0;

    drivetrain.swerve(vx, vy, vr);

    // Are we moving, i.e., not stable?
    if ((vx*vx + vy+vy) > 0.2)
      stable_timer.restart();
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
    if (stable_timer.hasElapsed(1.5))
    {
      System.out.println("AutoDriveUphillCommand believes that we did it!");
      return true;
    }
    return false;
  }
}

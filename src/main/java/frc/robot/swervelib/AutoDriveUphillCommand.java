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

  /** How long have we been trying? */
  private Timer timer = new Timer();

  /** Are we moving? */
  private boolean moving = false;

  public AutoDriveUphillCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    timer.restart();
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

    // Are we actually moving?
    moving = (vx*vx + vy+vy) > 0.2;
  }

  @Override
  public boolean isFinished()
  {
    // Start position may not be the correct field location,
    // but should be good enough to check how far we have moved since then.
    double distance = drivetrain.getPose().getTranslation().getDistance(start);
    // If we went more than 2 meters, assume something's wrong
    if (distance > 2.0)
    {
      System.out.println("Aborting AutoDriveUphillCommand, moved " + distance + " meters");
      return true;
    }
    // Are we not moving, and it's been a while, so we probably leveled out?
    if (! moving  &&  timer.hasElapsed(3.0))
    {
      System.out.println("AutoDriveUphillCommand believes that we did it!");
      return true;
    }
    return false;
  }
}

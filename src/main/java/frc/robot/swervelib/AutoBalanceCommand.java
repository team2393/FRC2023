// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Command for autonomous balance
 *
 *  Drive uphill (forward), expecting pitch to be greater than zero
 *  until pitch changes to either "balance" (~0) or "too far" (negative).
 *  Wait a little to allow charge station to settle down.
 *  If then pitched "down", drive back a fixed distance.
 */
public class AutoBalanceCommand extends SequentialCommandGroup
{
  /** At what pitch [deg] should we drive 1 m/s? */
  private static final double MAX_SPEED_ANGLE = 25.0;

  /** How far to back off [m] ? */
  private static final double BACKOFF = 0.15;
  // ^^ Path generation fails when trying to drive just 0.1m ...
  //    Would have to try TimedDriveCommand for very short distance..

  private final SwerveDrivetrain drivetrain;

  public AutoBalanceCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addCommands(new DriveUphill(),
                new WaitCommand(2),
                new DriveBack(),
                // Keep wheels sideways, no speed, to "brake", forever
                new TimedDriveCommand(drivetrain, 90, 0, 20000)
                // ,
                // new WaitCommand(2),
                // new AutoDriveUphillCommand(drivetrain)
                );
    addRequirements(drivetrain);
  }

  /** Drives uphill using pitch until we're "flat" or "downhill" */
  private class DriveUphill extends CommandBase
  {
    /** Start position */
    private Translation2d start;
    private double max_pitch;
    private boolean done;

    @Override
    public void initialize()
    {
      start = drivetrain.getPose().getTranslation();
      max_pitch = 0.0;
      done = false;
    }

    public void execute()
    {
      double pitch = drivetrain.getPitch();
      // Any "nose up" angle means we need to drive forward (X)
      double vx = pitch/MAX_SPEED_ANGLE;
      drivetrain.swerve(vx, 0, 0);

      if (pitch > max_pitch)
        max_pitch = pitch;
      if (pitch < 2.0)
      {
        System.out.println("Uphill pitch changed from max of " + max_pitch + " to " + pitch + " deg, stopping");
        done = true;
      }
      else
      { // If we moved more than 2 meters, assume something's wrong
        double distance = drivetrain.getPose().getTranslation().getDistance(start);
        if (distance > 2.0)
        {
          System.out.println("Aborting AutoDriveUphillCommand, moved " + distance + " meters");
          done = true;
        }
      }
    }

    @Override
    public boolean isFinished()
    {
      return done;
    }
  }

  /** Check pitch.
   *  If ~flat, run a command that does nothing.
   *  Else construct a command that runs backwards
   */
  private class DriveBack extends ProxyCommand
  {
    public DriveBack()
    {
      super(() ->
      {
        if (Math.abs(drivetrain.getPitch()) < 2.0)
          return Commands.none();

        Pose2d pose = drivetrain.getPose();
        Rotation2d back = pose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        double x = pose.getX() + BACKOFF*back.getCos();
        double y = pose.getY() + BACKOFF*back.getSin();

        System.out.println("Backing off from " + pose + " to " + x + ", " + y);
        return new SwerveToPositionCommand(drivetrain, x, y, pose.getRotation().getDegrees());
      });
    }
  }
}

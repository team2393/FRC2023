// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for driving at fixed angle and speed for some time */
public class TimedDriveCommand extends CommandBase
{
  private final Timer timer = new Timer();
  private final SwerveDrivetrain drivetrain;
  private final double angle, speed, seconds;

  public TimedDriveCommand(SwerveDrivetrain drivetrain, double angle, double speed, double seconds)
  {
    this.drivetrain = drivetrain;
    this.angle = angle;
    this.speed = speed;
    this.seconds = seconds;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    // In case we ever want to re-use
    // an instance of this command several times,
    // make sure the timer really starts over each time.
    timer.stop();
    timer.reset();
    timer.start();
  }

  public void execute()
  {
    drivetrain.drive(angle, speed);
  }

  @Override
  public boolean isFinished()
  {
    return timer.hasElapsed(seconds);
  }
}

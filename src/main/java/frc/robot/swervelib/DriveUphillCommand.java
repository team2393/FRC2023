// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for driving uphill */
public class DriveUphillCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;

  public DriveUphillCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public void execute()
  {
    // Start out with the controller inputs to allow "normal" swerving,
    // but their values would typically be zero since we're using the
    // up/down triggers
    double vx = SwerveOI.getForwardSpeed();
    double vy = SwerveOI.getLeftSpeed();
    double vr = SwerveOI.getRotationSpeed();

    // Check how fast we want to drive 'up/downhill':
    // Right trigger to move uphill, left trigger to move downhill
    double uphill = SwerveOI.joystick.getRightTriggerAxis() - SwerveOI.joystick.getLeftTriggerAxis();

    // Angles are reported in degrees.
    // If we're tilted 45 degrees, and the left or right trigger is pulled all the way in (uphill = -1 or 1),
    // we want to run at +-1 m/s uphill.

    // Any "nose up" angle means we need to drive forward
    vx += uphill * drivetrain.getPitch()/45.0;
    // Any "left up" angle means we need to drive left
    vy += uphill * drivetrain.getRoll()/45.0;

    drivetrain.swerve(vx, vy, vr);
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for swerving to a desired location
 * 
 *  Doesn't try to come up with a trajectory,
 *  simply uses vx, vy, vr to get to desired X, Y, Angle
 */
public class SwerveToPositionCommand extends CommandBase
{
  private static final Translation2d CENTER = new Translation2d();
  private final SwerveDrivetrain drivetrain;
 
  // kP = 0.4 m/s if 0.1 m away ... but limit speed to 0.5
  private final ProfiledPIDController x_pid = new ProfiledPIDController(4, 0, 0, new Constraints(0.5, 0.5));
  private final ProfiledPIDController y_pid = new ProfiledPIDController(4, 0, 0, new Constraints(0.5, 0.5));
  // kP = 10 deg/sec if 1 deg away .. 20 deg/sec
  private final ProfiledPIDController angle_pid = new ProfiledPIDController(10, 0, 0, new Constraints(20, 20));

  private boolean close_enough;
  
  /** @param drivetrain
   *  @param x Desired X [m],
   *  @param y Y [m],
   *  @param angle Angle [degrees]
   */
  public SwerveToPositionCommand(SwerveDrivetrain drivetrain, double x, double y, double angle)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    x_pid.setGoal(x);
    y_pid.setGoal(y);
    angle_pid.setGoal(angle);
  }

  @Override
  public void initialize()
  {
    Pose2d pose = drivetrain.getPose();
    x_pid.reset(pose.getX());
    y_pid.reset(pose.getY());
    angle_pid.reset(pose.getRotation().getDegrees());
    close_enough = false;
  }

  @Override
  public void execute()
  {
    Pose2d pose = drivetrain.getPose();
    double heading = pose.getRotation().getDegrees();
    double vx = x_pid.calculate(pose.getX());
    double vy = y_pid.calculate(pose.getY());
    double vr = angle_pid.calculate(heading);
    drivetrain.swerve(vx, vy, Math.toRadians(vr), CENTER);

    // Within 5 cm and 5 degrees of desired position?
    close_enough = Math.abs(angle_pid.getGoal().position - heading) < 5  &&
                   Math.abs(x_pid.getGoal().position - pose.getX()) < 0.05 &&
                   Math.abs(y_pid.getGoal().position - pose.getY()) < 0.05;

  }

  @Override
  public boolean isFinished()
  {
    return close_enough;
  }
}

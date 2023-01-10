// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for swerving to a desired location
 * 
 *  Doesn't try to come up with any trajectory,
 *  simply uses vx, vy, vr to get to desired X, Y, Angle
 */
public class SwerveToPositionCommand extends CommandBase
{
  private static final Translation2d CENTER = new Translation2d();
  private final Drivetrain drivetrain;
 
  // kP = 0.4 m/s if 0.1 m away ... but limit speed to 0.5
  private final ProfiledPIDController x_pid = new ProfiledPIDController(4, 0, 0, new Constraints(0.5, 0.5));
  private final ProfiledPIDController y_pid = new ProfiledPIDController(4, 0, 0, new Constraints(0.5, 0.5));
  // kP = 10 deg/sec if 1 deg away .. 20 deg/sec
  private final ProfiledPIDController angle_pid = new ProfiledPIDController(10, 0, 0, new Constraints(20, 20));
  
  /** @param drivetrain
   *  @param x Desired X [m],
   *  @param y Y [m],
   *  @param angle Angle [degrees]
   */
  public SwerveToPositionCommand(Drivetrain drivetrain, double x, double y, double angle)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    // Get within 5 cm of position
    x_pid.setTolerance(0.05);
    y_pid.setTolerance(0.05);
    x_pid.setGoal(x);
    y_pid.setGoal(y);
    // .. and 5 degrees
    angle_pid.setTolerance(5);
    angle_pid.setGoal(angle);
  }

  @Override
  public void initialize()
  {
    Pose2d pose = drivetrain.getPose();
    x_pid.reset(pose.getX());
    y_pid.reset(pose.getY());
    angle_pid.reset(pose.getRotation().getDegrees());
  }

  @Override
  public void execute()
  {
    Pose2d pose = drivetrain.getPose();
    double vx = Math.min(x_pid.calculate(pose.getX()), 0.5);
    double vy = Math.min(y_pid.calculate(pose.getY()), 0.5);
    double vr = Math.min(angle_pid.calculate(pose.getRotation().getDegrees()), 30);
    drivetrain.swerve(vx, vy, Math.toRadians(vr), CENTER);
  }

  @Override
  public boolean isFinished()
  {
    return x_pid.atGoal() &&
           y_pid.atGoal() &&
           angle_pid.atGoal();
  }
}

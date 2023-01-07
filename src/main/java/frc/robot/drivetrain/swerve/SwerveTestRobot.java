// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.CommandBaseRobot;

/** Swerve Test robot */
public class SwerveTestRobot extends CommandBaseRobot
{
  private final Drivetrain drivetrain = new Drivetrain();

  private Command drive = new DriveCommand(drivetrain);
  private Command swerve = new SwerveCommand(drivetrain);
  
  @Override
  public void teleopInit()
  {
    swerve.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    if (OI.joystick.getLeftBumperPressed())
      drive.schedule();
    if (OI.joystick.getRightBumperPressed())
      swerve.schedule();
  }

  @Override
  public void autonomousInit()
  {
    drivetrain.reset();

    TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
    List<Pose2d> path = List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(2.0, 0, Rotation2d.fromDegrees(0))
    );
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, config);
    Command auto = drivetrain.createTrajectoryCommand(trajectory, 45.0)
                             .andThen(new PrintCommand("Done"))
                             .andThen(new StayPutCommand(drivetrain, 45.0));

    auto.schedule();
  }
}

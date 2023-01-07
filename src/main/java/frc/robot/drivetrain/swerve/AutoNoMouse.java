// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandBaseRobot;

/** Auto-no-mouse routines */
public class AutoNoMouse
{
  private static final TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);

  /** Create trajectory from points
   *
   *  Given list of points must contain entries x, y, h,
   *  i.e., total length of x_y_h array must be a multiple of 3.
   *
   *  @param forward Are we driving forward?
   *  @param x_y_z Sequence of points { X, Y, Heading }
   */
  public static Trajectory createTrajectory(final boolean forward, final double... x_y_h)
  {
      if (x_y_h.length % 3 != 0)
          throw new IllegalArgumentException("List of { X, Y, Heading } contains " + x_y_h.length + " entries?!");

      List<Pose2d> waypoints = new ArrayList<>();
      for (int i=0; i<x_y_h.length; i += 3)
          waypoints.add(new Pose2d(x_y_h[i], x_y_h[i+1], Rotation2d.fromDegrees(x_y_h[i+2])));

      config.setReversed(! forward);
      return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  public static List<Command> createAutoCommands(Drivetrain drivetrain)
  {
    List<Command> autos = new ArrayList<>();
    
    {
      // Forward 2m, rotating to 45 degrees while moving
      // Wheels in the end stay at -45 deg
      Trajectory trajectory = createTrajectory(true,
                                               0.0, 0.0, 0.0,
                                               2.0, 0.0, 0.0);
      Command auto = drivetrain.createTrajectoryCommand(trajectory, 45.0)
                               .andThen(new PrintCommand("Done"))
                               .andThen(new StayPutCommand(drivetrain, -45.0));
      auto.setName("Forward 2m");
      autos.add(auto);
    }

    {
      // Forward 1m, then back
      SequentialCommandGroup auto = new SequentialCommandGroup();
      Trajectory forward = createTrajectory(true,
                                            0.0, 0.0, 0.0,
                                            1.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(forward, 0.0));
      Trajectory back = createTrajectory(false,
                                            1.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back, 0.0));
      auto.addCommands(new PrintCommand("Done"));
      auto.addCommands(new StayPutCommand(drivetrain, 180.0));
      auto.setName("Fw 1 and back");
      autos.add(auto);
    }

    return autos;
  }
}

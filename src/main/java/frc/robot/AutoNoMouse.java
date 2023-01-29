// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
import frc.robot.swervelib.ResetPositionCommand;
import frc.robot.swervelib.StayPutCommand;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveToPositionCommand;
import frc.robot.swervelib.TimedDriveCommand;

/** Auto-no-mouse routines */
public class AutoNoMouse {
  // Run at up to 1.0m/s, accelerate by 0.5ms per second
  private static final TrajectoryConfig config = new TrajectoryConfig(1.0, 0.5);

  /**
   * Create trajectory from points
   *
   * Given list of points must contain entries x, y, h,
   * i.e., total length of x_y_h array must be a multiple of 3.
   *
   * @param forward Are we driving forward?
   * @param x_y_z   Sequence of points { X, Y, Heading }
   */
  public static Trajectory createTrajectory(final boolean forward, final double... x_y_h) {
    if (x_y_h.length % 3 != 0)
      throw new IllegalArgumentException("List of { X, Y, Heading } contains " + x_y_h.length + " entries?!");

    List<Pose2d> waypoints = new ArrayList<>();
    for (int i = 0; i < x_y_h.length; i += 3)
      waypoints.add(new Pose2d(x_y_h[i], x_y_h[i + 1], Rotation2d.fromDegrees(x_y_h[i + 2])));

    config.setReversed(!forward);
    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  /** Create all our auto-no-mouse commands */
  public static List<Command> createAutoCommands(SwerveDrivetrain drivetrain) {
    List<Command> autos = new ArrayList<>();

    {
      Command auto = new StayPutCommand(drivetrain, 0.0);
      auto.setName("Stay");
      autos.add(auto);
    }

    {
      Command auto = new SwerveToPositionCommand(drivetrain, 0, 0, 0)
          .andThen(new PrintCommand("Back HOME!"));
      auto.setName("Home");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      for (int i = 0; i < 5; ++i) {
        auto.addCommands(new TimedDriveCommand(drivetrain, 0, 0, 3));
        auto.addCommands(new TimedDriveCommand(drivetrain, 15, 0, 3));
      }
      auto.setName("Wiggle 15");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      for (int i = 0; i < 5; ++i) {
        auto.addCommands(new TimedDriveCommand(drivetrain, 0, 0, 3));
        auto.addCommands(new TimedDriveCommand(drivetrain, 90, 0, 3));
      }
      auto.setName("Wiggle 90");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      for (int i = 0; i < 5; ++i) {
        auto.addCommands(new TimedDriveCommand(drivetrain, 0, 0, 3));
        auto.addCommands(new TimedDriveCommand(drivetrain, 180, 0, 3));
      }
      auto.setName("Wiggle 180");
      autos.add(auto);
    }

    {
      Trajectory trajectory = createTrajectory(true,
          0.0, 0.0, 0.0,
          3.9, 0.0, 45.0,
          4.5, 1.3, 90.0,
          3.5, 2.3, 180.0,
          1.9, 1.9, 180.0);
      Command auto = new ResetPositionCommand(drivetrain)
          .andThen(drivetrain.createTrajectoryCommand(trajectory, 180.0))
          .andThen(new PrintCommand("Done"))
          .andThen(new StayPutCommand(drivetrain, 0.0));
      auto.setName("U-Cube");
      autos.add(auto);
    }

    {
      // Forward and right
      Trajectory trajectory = createTrajectory(true,
          0.0, 0.0, 0.0,
          3.2, 2.0, 0.0,
          4.5, 1.5, -90.0,
          3.5, -0.5, -90.0);
      Command auto = new ResetPositionCommand(drivetrain)
          .andThen(drivetrain.createTrajectoryCommand(trajectory, -90.0))
          .andThen(new PrintCommand("Done"))
          .andThen(new StayPutCommand(drivetrain, 0.0));
      auto.setName("Test Trajectory");
      autos.add(auto);
    }

    {
      // Forward 2m, rotating to 45 degrees while moving
      // Wheels in the end stay at -45 deg
      Trajectory trajectory = createTrajectory(true,
          0.0, 0.0, 0.0,
          2.0, 0.0, 0.0);
      Command auto = new ResetPositionCommand(drivetrain)
          .andThen(drivetrain.createTrajectoryCommand(trajectory, 45.0))
          .andThen(new PrintCommand("Done"))
          .andThen(new StayPutCommand(drivetrain, -45.0));
      auto.setName("Forward 2m");
      autos.add(auto);
    }

    {
      // Forward 1m, then back
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new ResetPositionCommand(drivetrain));
      Trajectory forward = createTrajectory(true,
          0.0, 0.0, 0.0,
          1.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(forward, 0.0));
      Trajectory back = createTrajectory(false,
          1.0, 0.0, 0.0,
          0.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back, 0.0));
      auto.addCommands(new StayPutCommand(drivetrain, 180.0));
      auto.setName("Fw 1 and back");
      autos.add(auto);
    }

    {
      // Little forward and to right, then back
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new ResetPositionCommand(drivetrain));
      Trajectory forward = createTrajectory(true,
          0.0, 0.0, 0.0,
          0.7, -0.7, -45.0,
          0.9, -2.7, -90);
      auto.addCommands(drivetrain.createTrajectoryCommand(forward, -90.0));
      Trajectory back = createTrajectory(false,
          0.9, -2.7, -90.0,
          0.9, 0.0, -90.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back, 0.0));
      Trajectory back2 = createTrajectory(false,
          0.9, 0.0, 0.0,
          0.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back2, 0.0));
      auto.addCommands(new StayPutCommand(drivetrain, 0.0));
      auto.setName("|----");
      autos.add(auto);
    }

    return autos;
  }
}

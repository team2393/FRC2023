// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.swervelib.AutoDriveUphillCommand;
import frc.robot.swervelib.SelectAbsoluteTrajectoryCommand;
import frc.robot.swervelib.SelectRelativeTrajectoryCommand;
import frc.robot.swervelib.StayPutCommand;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveToPositionCommand;
import frc.robot.swervelib.TimedDriveCommand;
import frc.robot.swervelib.VariableWaitCommand;

/** Auto-no-mouse routines */
public class AutoNoMouse
{
  // Run at up to 1.0m/s, accelerate by 0.5ms per second
  private static final TrajectoryConfig config = new TrajectoryConfig(1.5, 1);

  /** Width of field */
  private static final double WIDTH = 16.541748046875;

  /** Create trajectory from points
   *
   *  Given list of points must contain entries x, y, h,
   *  i.e., total length of x_y_h array must be a multiple of 3.
   *
   *  @param forward Are we driving forward?
   *  @param x_y_z   Sequence of points { X, Y, Heading }
   */
  public static Trajectory createTrajectory(final boolean forward, final double... x_y_h)
  {
    if (x_y_h.length % 3 != 0)
      throw new IllegalArgumentException("List of { X, Y, Heading } contains " + x_y_h.length + " entries?!");

    List<Pose2d> waypoints = new ArrayList<>();
    for (int i = 0; i < x_y_h.length; i += 3)
      waypoints.add(new Pose2d(x_y_h[i], x_y_h[i + 1], Rotation2d.fromDegrees(x_y_h[i + 2])));

    config.setReversed(!forward);
    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  /** Create command that follows a PathWeaver path 
   *  @param drivetrain Drivetrain to use
   *  @param pathname Base name "XXX" for "deploy/output.XXX.wpilib.json"
   *  @param final_heading .. of robot
   *  @return Command that follows the path
   */
  public static Command followPathWeaver(SwerveDrivetrain drivetrain, String pathname, double final_heading)
  {
    Path file = Filesystem.getDeployDirectory().toPath().resolve("output").resolve(pathname + ".wpilib.json");
    try
    {
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(file);
      return drivetrain.createTrajectoryCommand(trajectory, final_heading);
    }
    catch (Exception ex)
    {
      System.err.println("Cannot load " + file);
      ex.printStackTrace();
    }
    return new PrintCommand("Error loading ");
  }

  /** Create all our auto-no-mouse commands */
  public static List<Command> createAutoCommands(SwerveDrivetrain drivetrain)
  {
    final List<Command> autos = new ArrayList<>();

    {
      autos.add(new SwerveToPositionCommand(drivetrain, 0, 0, 0)
                    .andThen(new PrintCommand("Back HOME!"))
                    .withName("Home"));
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      for (int i = 0; i < 5; ++i)
      {
        auto.addCommands(new TimedDriveCommand(drivetrain, 0, 0, 3));
        auto.addCommands(new TimedDriveCommand(drivetrain, 15, 0, 3));
      }
      auto.setName("Wiggle 15");
      autos.add(auto);
    }

    {
      Trajectory trajectory = createTrajectory(true,  0.0, 0.0, 0.0,
                                                      3.9, 0.0, 45.0,
                                                      4.5, 1.3, 90.0,
                                                      3.5, 2.3, 180.0,
                                                      1.9, 1.9, 180.0);
      autos.add(new SelectRelativeTrajectoryCommand(drivetrain)
                    .andThen(drivetrain.createTrajectoryCommand(trajectory, 180.0))
                    .andThen(new PrintCommand("Done"))
                    .andThen(new StayPutCommand(drivetrain, 0.0))
                    .withName("U-Cube"));
    }

    {
      // Forward 1m, then back
      SequentialCommandGroup auto = new SequentialCommandGroup();
      // auto.addCommands(new ResetPositionCommand(drivetrain));
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory forward = createTrajectory(true, 0.0, 0.0, 0.0,
                                                  1.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(forward, 0.0));
      Trajectory back = createTrajectory(false, 1.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back, 0.0));
      auto.addCommands(new StayPutCommand(drivetrain, 180.0));
      auto.setName("Fw 1 and back");
      autos.add(auto);
    }

    {
      // Little forward and to right, then back
      // from current location on
      SequentialCommandGroup auto = new SequentialCommandGroup();
      // auto.addCommands(new ResetPositionCommand(drivetrain));
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory forward = createTrajectory(true, 0.0, 0.0, 0.0,
                                                  0.7, -0.7, -45.0,
                                                  0.9, -2.7, -90);
      auto.addCommands(drivetrain.createTrajectoryCommand(forward, -90.0));
      Trajectory back = createTrajectory(false, 0.9, -2.7, -90.0,
                                                0.9, 0.0, -90.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back, 0.0));
      Trajectory back2 = createTrajectory(false,  0.9, 0.0, 0.0,
                                                  0.0, 0.0, 0.0);
      auto.addCommands(drivetrain.createTrajectoryCommand(back2, 0.0));
      auto.addCommands(new StayPutCommand(drivetrain, 0.0));
      auto.setName("|----");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 1, 1, 45));
      auto.addCommands(new PrintCommand("Pretend we do something"));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 2, 2, 90));
      auto.addCommands(new PrintCommand("Pretend we do something else"));
      auto.addCommands(new SwerveToPositionCommand(drivetrain, 3, 3, 180));
      auto.setName("Points");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 1.85, 5.05, 0,
                                               5.00, 4.70, 0);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 0));
      auto.setName("BTE");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 1.85, 2.75, 45,
                                               2.50, 4.60, 0,
                                               6.43, 4.60, 0);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 0));
      auto.addCommands(new PrintCommand("Pickup"));
      auto.addCommands(new WaitCommand(2));
      path = createTrajectory(true, 6.43, 4.60, 180,
                                    3.80, 4.60, 180,
                                    1.90, 4.45, 180);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 180));
      auto.setName("BMR");
      autos.add(auto);
    }

    { // Blue Bottom Drop item then Retrieve another
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new PrintCommand("Dropping..."));
      auto.addCommands(new WaitCommand(2));
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 1.93, 0.5, 180));
      Trajectory path = createTrajectory(true, 1.93, 0.50, 0,
                                               5.42, 0.9, 0,
                                               6.39, 2.14, 0);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 0));
      auto.setName("BBDR");
      autos.add(auto);
    }

    { // 'Mirrored' sequence: X   -->  WIDTH-X,   heading --> 180-heading.  Y stays.
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new PrintCommand("Dropping..."));
      auto.addCommands(new WaitCommand(2));
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, WIDTH-1.93, 0.5, 180-180));
      Trajectory path = createTrajectory(true, WIDTH-1.93, 0.50, 180-0,
                                               WIDTH-5.42, 0.9,  180-0,
                                               WIDTH-6.39, 2.14, 180-0);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 180-0));
      auto.setName("RBDR");
      autos.add(auto);
    }

    { // Example for using PathWeaver
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 1.66, 4.47, 0));
      auto.addCommands(followPathWeaver(drivetrain, "Circle", 0));
      auto.setName("PWCircle");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 1.66, 4.47, 0));
      auto.addCommands(followPathWeaver(drivetrain, "Test", 0));
      auto.setName("PWTest");
      autos.add(auto);
    }

    {
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain, 1.84, 1.12, 180));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      Trajectory path = createTrajectory(true, 1.84, 1.12,  0,
                                               6,    1.1,  45,
                                               6,    2.4,  135,
                                               4.5,  2.85, 180);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 180));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      auto.setName("BalanceTest");
      autos.add(auto);
    }

    { // Skeleton for another auto option
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));

      auto.setName("Another");
      autos.add(auto);
    }

    return autos;
  }
}

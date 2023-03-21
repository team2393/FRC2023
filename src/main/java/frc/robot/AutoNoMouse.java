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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.magnussen.GrabberEjectCommand;
import frc.robot.magnussen.charm.Charm;
import frc.robot.magnussen.charm.ExtendArmCommand;
import frc.robot.magnussen.charm.RetractArmCommand;
import frc.robot.magnussen.charm.SetArmCommand;
import frc.robot.swervelib.AutoBalanceCommand;
import frc.robot.swervelib.AutoDriveUphillCommand;
import frc.robot.swervelib.SelectAbsoluteTrajectoryCommand;
import frc.robot.swervelib.SelectRelativeTrajectoryCommand;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.VariableWaitCommand;

/** Auto-no-mouse routines */
public class AutoNoMouse
{
  // Run at up to 1.5m/s, accelerate by 1.0ms per second
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
  public static List<Command> createAutoCommands(SwerveDrivetrain drivetrain, Charm coordinator)
  {
    final List<Command> autos = new ArrayList<>();

    // ---------------------- Move out -----------------------------------

    { // Simply drive forward 1.5 m, can be used from Blue or Red, Top or Bottom
      SequentialCommandGroup auto = new SequentialCommandGroup();
      auto.setName("Forward 1.5m");
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectRelativeTrajectoryCommand(drivetrain));
      Trajectory path = createTrajectory(true, 0, 0, 0,
                                               1.50, 0, 0);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 0));
      autos.add(auto);
    }

    { // Blue, Top node, Exit
      SequentialCommandGroup auto = new SequenceWithStart("BTE",  1.84, 4.45, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(followPathWeaver(drivetrain, "BTE", 0));
      autos.add(auto);
    }

    { // Red, Top node, Exit
      SequentialCommandGroup auto = new SequenceWithStart("RTE",  14.75, 4.45, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(followPathWeaver(drivetrain, "RTE", 180));
      autos.add(auto);
    }

    { // Blue, Middle node, Exit
      SequentialCommandGroup auto = new SequenceWithStart("BME",  1.84, 2.7, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(followPathWeaver(drivetrain, "BME", 0));
      autos.add(auto);
    }

    { // Red, Middle node, Exit
      SequentialCommandGroup auto = new SequenceWithStart("RME",  14.9, 2.8, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(followPathWeaver(drivetrain, "RME", 180));
      autos.add(auto);
    }
    
    { // Blue, Bottom node, Exit
      SequentialCommandGroup auto = new SequenceWithStart("BBE",  1.6, 1.03, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(followPathWeaver(drivetrain, "BBE", 0));
      autos.add(auto);
    }

     { // Red, Bottom node, Exit
      SequentialCommandGroup auto = new SequenceWithStart("RBE",  14.9, 1.05, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(followPathWeaver(drivetrain, "RBE", 180));
      autos.add(auto);
    }
    // ---------------------- Move out and Balance -----------------------------------

    { // Blue, Top node, drive out then Balance
      SequentialCommandGroup auto = new SequenceWithStart("BTBalance",  1.84, 4.45, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      auto.addCommands(followPathWeaver(drivetrain, "BTBalance", 180));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      // TODO Replace all AutoDriveUphillCommand(drivetrain) with AutoBalanceCommand(drivetrain, false)?
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      autos.add(auto);
    }

    { // Red, Top node, drive out then Balance
      SequentialCommandGroup auto = new SequenceWithStart("RTBalance",  WIDTH-1.84, 4.45, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      auto.addCommands(followPathWeaver(drivetrain, "RTBalance", 0));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      autos.add(auto);
    }

    { // Blue, Middle node, drive out then Balance
      SequentialCommandGroup auto = new SequenceWithStart("BMBalance",  1.84, 2.75, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      auto.addCommands(followPathWeaver(drivetrain, "BMBalance", 180));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      autos.add(auto);
    }

    { // Red, Middle node, drive out then Balance
      SequentialCommandGroup auto = new SequenceWithStart("RMBalance",  14.9, 2.8, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      auto.addCommands(followPathWeaver(drivetrain, "RMBalance", 0));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      autos.add(auto);
    }
    
    { // Blue, Bottom node, drive out then Balance
      SequentialCommandGroup auto = new SequenceWithStart("BBBalance", 1.84, 1.12, 180);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      Trajectory path = createTrajectory(true, 1.84, 1.12,  0,
                                               6,    1.3,  45,
                                               6,    2.4,  135,
                                               4.5,  2.85, 180);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 180));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      autos.add(auto);
    }

    { // Red, Bottom node, drive out then Balance
      SequentialCommandGroup auto = new SequenceWithStart("RBBalance",  14.78, 1.05, 0);
      auto.addCommands(new VariableWaitCommand());
      auto.addCommands(new SelectAbsoluteTrajectoryCommand(drivetrain));
      auto.addCommands(new PrintCommand("Driving to charge station..."));
      auto.addCommands(followPathWeaver(drivetrain, "RBBalance", 0));
      auto.addCommands(new PrintCommand("Driving uphill .."));
      auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new PrintCommand("Done!"));
      autos.add(auto);
    }

    // ---------------------- Other -----------------------------------

    { // blue or red Middle node, Drop cube, Out, Balance
      SequentialCommandGroup auto = new SequenceWithStart("MDOB", 1.89, 2.88, 0);
      // Prepare to drop cube into middle node
      auto.addCommands(new SetArmCommand(coordinator, -200));
      auto.addCommands(new ExtendArmCommand(coordinator));
      // TODO Wait for extension, but shorter once piston gets more pressure
      auto.addCommands(new WaitCommand(0.5));
      
      Trajectory path = createTrajectory(true, 1.89, 2.88, 0,
                                               6.75, 2.88, 0);
      auto.addCommands(
        new ParallelCommandGroup(
          new SequentialCommandGroup(// Eject & pull arm back in
                                     new ProxyCommand(new GrabberEjectCommand(coordinator.grabber)),
                                     new RetractArmCommand(coordinator),
                                     new SetArmCommand(coordinator, -155)),
          new SequentialCommandGroup(// Drive out over the charge station
                                     new SelectAbsoluteTrajectoryCommand(drivetrain),
                                     drivetrain.createTrajectoryCommand(path, 0))));
      // Drive back onto the charge station .. and balance
      path = createTrajectory(true, 6.75, 2.88, 180,
                                    4.8,  2.88, 180);
      auto.addCommands(drivetrain.createTrajectoryCommand(path, 0));
      auto.addCommands(new AutoBalanceCommand(drivetrain, true));
      autos.add(auto);

      // Could write like this. Is that better?
      // auto.addCommands(new SetArmCommand(coordinator, -200),
      //                  new ExtendArmCommand(coordinator),
      //                  new WaitCommand(1),
      //                  Commands.parallel(new GrabberEjectCommand(coordinator.grabber).asProxy()
      //                                       .andThen(new RetractArmCommand(coordinator))
      //                                       .andThen(new SetArmCommand(coordinator, -155)),
      //                                    new SelectAbsoluteTrajectoryCommand(drivetrain)
      //                                        .andThen(drivetrain.createTrajectoryCommand(path1, 0))
      //                                   ),
      //                  drivetrain.createTrajectoryCommand(path2, 0),
      //                  new AutoBalanceCommand(drivetrain, true));
    }

    { // Skeleton for another auto option
      SequentialCommandGroup auto = new SequenceWithStart("Balance", 5.19, 2.88, 180);
      auto.addCommands(new VariableWaitCommand());
      // auto.addCommands(new AutoDriveUphillCommand(drivetrain));
      auto.addCommands(new AutoBalanceCommand(drivetrain, false));
      autos.add(auto);
    }

    return autos;
  }
}

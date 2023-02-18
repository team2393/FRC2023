// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.AutoNoMouse;
import frc.robot.CommandBaseRobot;
import frc.robot.swervelib.RelativeSwerveCommand;
import frc.robot.swervelib.ResetPositionCommand;
import frc.robot.swervelib.SwerveOI;
import frc.robot.vision.LimelightClient;

/** Team 2393 2023 Robot */
public class MagnussenRobot extends CommandBaseRobot
{
  private final MagnussenDriveTrain drivetrain = new MagnussenDriveTrain();
  private final CommandBase drive_relative = new RelativeSwerveCommand(drivetrain, false);
  private final CommandBase reset = new ResetPositionCommand(drivetrain);

  private final SendableChooser<Command> autos = new SendableChooser<>();

  // TODO private final TheGreatCoordinator coordinator = new TheGreatCoordinator(true);

  private final Pneumatics pneumatics = new Pneumatics();

  private LimelightClient camera;

  @Override
  public void robotInit()
  {
    super.robotInit();

    OI.reset();

    autos.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    for (Command auto : AutoNoMouse.createAutoCommands(drivetrain))
      autos.addOption(auto.getName(), auto);
    SmartDashboard.putData("Auto Options", autos);

    // TODO Configure power dist. & publish power info
    // PowerDistribution power = new PowerDistribution(1, ModuleType.kRev);
    // power.clearStickyFaults();
    // power.setSwitchableChannel(false);
    // SmartDashboard.putData(power);

    camera = new LimelightClient(drivetrain);
  }

  @Override
  public void disabledInit()
  {
    // Make robot easier to move while disabled
    drivetrain.brake(false);
  }

  @Override
  public void teleopInit()
  {
    // Stop motors unless they're supposed to move
    drivetrain.brake(true);
    drive_relative.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    // Activate different drive mode?
    if (SwerveOI.selectRelativeMode())
      drive_relative.schedule();
    if (SwerveOI.resetOrigin())
        reset.schedule();

    // TODO coordinator.run();
  }

  @Override
  public void autonomousInit()
  {
    // Stop motors unless they're supposed to move
    drivetrain.brake(true);

    // Run selected auto
    autos.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // Nothing to do but let command run
  }
}

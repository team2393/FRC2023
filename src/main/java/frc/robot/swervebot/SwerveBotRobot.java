package frc.robot.swervebot;
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.CommandBaseRobot;
import frc.robot.swervelib.AbsoluteSwerveCommand;
import frc.robot.swervelib.FixedSpeedCommand;
import frc.robot.swervelib.RelativeSwerveCommand;
import frc.robot.swervelib.ResetPositionCommand;
import frc.robot.swervelib.SwerveOI;

/** ServeBot */
public class SwerveBotRobot extends CommandBaseRobot
{
  private final SwervebotDrivetrain drivetrain = new SwervebotDrivetrain();
  // private final CommandBase drive = new DriveCommand(drivetrain);
  private final CommandBase drive_relative = new RelativeSwerveCommand(drivetrain);
  private final CommandBase drive_absolute = new AbsoluteSwerveCommand(drivetrain);
  private final CommandBase fixed_speed = new FixedSpeedCommand(drivetrain, 0.3);
  private final CommandBase reset = new ResetPositionCommand(drivetrain);

  private final SendableChooser<Command> autos = new SendableChooser<>();

  @Override
  public void robotInit()
  {
    super.robotInit();

    autos.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    for (Command auto : AutoNoMouse.createAutoCommands(drivetrain))
      autos.addOption(auto.getName(), auto);
    SmartDashboard.putData("Auto Options", autos);
  }

  @Override
  public void teleopInit()
  {
    drive_relative.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    // Activate different drive mode?
    if (SwerveOI.selectAbsoluteMode())
      drive_absolute.schedule();
    if (SwerveOI.selectRelativeMode())
      drive_relative.schedule();
    if (SwerveOI.resetOrigin())
        reset.schedule();
    if (SwerveOI.selectFixedSpeed())
        fixed_speed.schedule();
    else
        fixed_speed.cancel();
  }

  @Override
  public void autonomousInit()
  {
    autos.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // Nothing to do but let command run
  }
}

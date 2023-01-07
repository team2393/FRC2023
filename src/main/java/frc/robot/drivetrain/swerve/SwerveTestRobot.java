// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.CommandBaseRobot;

/** Swerve Test robot */
public class SwerveTestRobot extends CommandBaseRobot
{
  private final Drivetrain drivetrain = new Drivetrain();

  private Command drive = new DriveCommand(drivetrain);
  private Command relative_swerve = new RelativeSwerveCommand(drivetrain);
  private Command absolute_swerve = new AbsoluteSwerveCommand(drivetrain);
  private SendableChooser<Command> autos = new SendableChooser<>();

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
    OI.reset();
    relative_swerve.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    if (OI.selectAbsoluteMode())
    {
      System.out.println("ABSOLUTE");
      absolute_swerve.schedule();
    }
    else if (OI.selectRelativeMode())
    {
      System.out.println("RELATIVE");
      relative_swerve.schedule();
    }
  }

  @Override
  public void autonomousInit()
  {
    drivetrain.reset();
    autos.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // Let auto command run...  
  }
}

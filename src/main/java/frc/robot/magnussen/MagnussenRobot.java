// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.AutoNoMouse;
import frc.robot.CommandBaseRobot;
import frc.robot.swervelib.AbsoluteSwerveCommand;
import frc.robot.swervelib.FixedSpeedCommand;
import frc.robot.swervelib.RelativeSwerveCommand;
import frc.robot.swervelib.ResetPositionCommand;
import frc.robot.swervelib.SwerveOI;

// TODO:
//
// - Update all motor firmwares
// - 'blink' all motors, set CAN ID, update RobotMap
// - Deploy robot, briefly POV up for 'forward' and release to enter 'StayPul',
//   adjust all swerve angle offsets
// - Run Wiggle XX autos to adjust rotator settings
// - Hold POV up and adjust driver settings to get correct speed

/** Team 2393 2023 Robot */
public class MagnussenRobot extends CommandBaseRobot
{
  private final MagnussenDriveTrain drivetrain = new MagnussenDriveTrain();
  // private final CommandBase drive = new DriveCommand(drivetrain);
  private final CommandBase drive_relative = new RelativeSwerveCommand(drivetrain);
  private final CommandBase drive_absolute = new AbsoluteSwerveCommand(drivetrain);
  private final CommandBase fixed_fwd = new FixedSpeedCommand(drivetrain, 0.2);
  private final CommandBase fixed_back = new FixedSpeedCommand(drivetrain, -0.2);
  private final CommandBase reset = new ResetPositionCommand(drivetrain);

  private final SendableChooser<Command> autos = new SendableChooser<>();

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
    // PowerDistribution power = new PowerDistribution(0, ModuleType.kRev);
    // power.clearStickyFaults();
    // power.setSwitchableChannel(false);
    // SmartDashboard.putData(power);
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
    if (SwerveOI.selectAbsoluteMode())
      drive_absolute.schedule();
    if (SwerveOI.selectRelativeMode())
      drive_relative.schedule();
    if (SwerveOI.resetOrigin())
        reset.schedule();
    if (SwerveOI.selectFixedForward())
        fixed_fwd.schedule();
    else if (SwerveOI.selectFixedBack())
        fixed_back.schedule();
    else
    {
        fixed_fwd.cancel();
        fixed_back.cancel();
    }
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

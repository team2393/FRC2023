// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.AutoNoMouse;
import frc.robot.CommandBaseRobot;
import frc.robot.SequenceWithStart;
import frc.robot.led.FillCommand;
import frc.robot.led.GreenGoldBlink;
import frc.robot.led.LED;
import frc.robot.led.MovingColorList;
import frc.robot.led.RainbowCommand;
import frc.robot.magnussen.charm.Charm;
import frc.robot.swervelib.DriveUphillCommand;
import frc.robot.swervelib.RelativeSwerveCommand;
import frc.robot.swervelib.ResetPositionCommand;
import frc.robot.vision.LimelightClient;

/** Team 2393 2023 Robot */
public class MagnussenRobot extends CommandBaseRobot
{
  private final MagnussenDriveTrain drivetrain = new MagnussenDriveTrain();
  private final CommandBase drive_relative = new RelativeSwerveCommand(drivetrain, false);
  private final CommandBase drive_uphill = new DriveUphillCommand(drivetrain);
  private final CommandBase reset = new ResetPositionCommand(drivetrain);

  private final SendableChooser<Command> autos = new SendableChooser<>();

  // private final TheGreatCoordinator coordinator = new TheGreatCoordinator();
  private final Charm coordinator = new Charm();

  private final Pneumatics pneumatics = new Pneumatics();

  private LimelightClient camera;

  // LED and several patterns
  private LED led = new LED();
  private CommandBase auto_led = new RainbowCommand(led),
                      normal_led, // Set in teleopInit based on alliance
                      loaded_led = new MovingColorList(led, 10, Color.kWhite, Color.kRed, Color.kBlue);

  @Override
  public void robotInit()
  {
    super.robotInit();

    OI.reset();

    autos.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    for (Command auto : AutoNoMouse.createAutoCommands(drivetrain, coordinator))
      autos.addOption(auto.getName(), auto);
    SmartDashboard.putData("Auto Options", autos);

    // Configure power dist. & publish power info
    PowerDistribution power = new PowerDistribution(1, ModuleType.kRev);
    power.clearStickyFaults();
    power.setSwitchableChannel(false);
    SmartDashboard.putData(power);

    // camera = new LimelightClient(drivetrain);

    // Default command, will be (re-) activated when disabled
    led.setDefaultCommand(new GreenGoldBlink(led));
  }

  @Override
  public void disabledInit()
  {
    // Make robot easier to move while disabled
    drivetrain.brake(false);

    led.getDefaultCommand().schedule();
  }

  @Override
  public void disabledPeriodic()
  {
    // Is an auto option selected that's of type
    // SequenceWithStart?
    Command auto = autos.getSelected();
    if (auto instanceof SequenceWithStart)
    { // If so, update odometry to show that
      Pose2d start = ((SequenceWithStart) auto).getStart();
      drivetrain.setOdometry(start.getX(), start.getY(), start.getRotation().getDegrees());
    }
  }

  @Override
  public void teleopInit()
  {
    OI.reset();
    // Stop motors unless they're supposed to move
    drivetrain.brake(true);
  
    drive_relative.schedule();

    // Pick pattern based on alliance
    normal_led = DriverStation.getAlliance() == Alliance.Red
               ? new FillCommand(led, Color.kFirstRed)
               : new FillCommand(led, Color.kFirstBlue);
  }

  @Override
  public void teleopPeriodic()
  {
    // Pick 'loaded' or normal pattern
    if (coordinator.grabber.haveGamepiece())
      loaded_led.schedule();
    else
      normal_led.schedule();

    // Activate different drive mode?
    if (OI.selectNormalDriveMode())
      drive_relative.schedule();
    else if (OI.selectUphillMode())
    {
      drive_uphill.schedule();
    }
    
    if (OI.resetOrigin())
        reset.schedule();

    boolean normal = !drive_uphill.isScheduled();
    
    if (OI.selectSubstationIntakeMode()  &&  normal)
      coordinator.intakeFromSubstation(OI.selectCubeIntake());

    if (OI.selectNearNodeMode() && normal)
      coordinator.near();

    if (OI.selectMiddleNodeMode() && normal)
      coordinator.mid();

    if (OI.selectFarNodeMode() && normal)
      coordinator.far();
      
    if (OI.ejectGamepiece())
      coordinator.eject();
  }

  @Override
  public void autonomousInit()
  {
    // Stop motors unless they're supposed to move
    drivetrain.brake(true);

    // Run selected auto
    autos.getSelected().schedule();

    auto_led.schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // Nothing to do but let command run
  }
}

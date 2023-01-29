// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command for swerving while keeping the heading */
public class DriveStraightCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;
  private NetworkTableEntry nt_p;
  private double heading;

  public DriveStraightCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    nt_p = SmartDashboard.getEntry("Straight P");
    nt_p.setDefaultNumber(0.1);
  }

  @Override
  public void initialize()
  {
    heading = drivetrain.getHeading().getDegrees();
  }

  public void execute()
  {
    // Proportional correction to keep initial heading
    double error = Math.IEEEremainder(heading - drivetrain.getHeading().getDegrees(), 360.0);
    // error is in degrees, vr in radians/sec, so 'P' tends to be small number
    double vr = error * nt_p.getDouble(0.0);

    drivetrain.swerve(SwerveOI.getForwardSpeed(), SwerveOI.getLeftSpeed(), vr);
  }
}

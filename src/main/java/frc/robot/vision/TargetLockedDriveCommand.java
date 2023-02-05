// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swervelib.SwerveDrivetrain;
import frc.robot.swervelib.SwerveOI;

/** Drive command that overrides rotation by camera target */
public class TargetLockedDriveCommand extends CommandBase
{
  private final SwerveDrivetrain drivetrain;
  private NetworkTableEntry nt_pipeline;
  private NetworkTableEntry nt_valid;
  private NetworkTableEntry nt_x;
  private NetworkTableEntry nt_P;
  private NetworkTableEntry nt_max;

  public TargetLockedDriveCommand(SwerveDrivetrain drivetrain)
  {
    this.drivetrain = drivetrain;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    nt_pipeline = table.getEntry("pipeline");
    nt_valid = table.getEntry("tv");
    nt_x = table.getEntry("tx");
    nt_P = SmartDashboard.getEntry("LockedDriveP");
    nt_P.setDefaultDouble(0.05);
    nt_max = SmartDashboard.getEntry("LockedDriveMax");
    nt_max.setDefaultDouble(Math.toRadians(90));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize()
  {
    nt_pipeline.setNumber(1);
  }
  
  public void execute()
  {
    // Start by using joystick to rotate
    double rot = SwerveOI.getRotationSpeed();
    if (nt_valid.getInteger(0) > 0)
    { // If we have camera info, use that to aim onto target.
      // X will be negative if target is to the left,
      // and we would have to rotate by positive angle to compensate
      rot -= nt_P.getDouble(0.0) * nt_x.getDouble(0.0);
      // Limit rotational speed
      double max = nt_max.getDouble(1.0);
      rot = MathUtil.clamp(rot, -max, max);
    }
    drivetrain.swerve(SwerveOI.getForwardSpeed(),
                      SwerveOI.getLeftSpeed(),
                      rot);
  }

  @Override
  public void end(boolean interrupted)
  {
    nt_pipeline.setNumber(0);
  }
}

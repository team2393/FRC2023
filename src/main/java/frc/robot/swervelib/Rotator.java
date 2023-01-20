package frc.robot.swervelib;
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Rotational part of a swerve module */
abstract public class Rotator
{
  private final NetworkTableEntry nt_offset;
  private final NetworkTableEntry nt_P;
  private final NetworkTableEntry nt_angle;
  private final NetworkTableEntry nt_desired;
  private double simulated_angle = 0.0;

  /** Construct Rotator
   *  @param index Rotator index 0..3
   *  @param offset Offset from 'forward' in degrees
   */
  public Rotator(int index, double offset)
  {
    nt_offset = SmartDashboard.getEntry("Offset" + index);
    nt_angle = SmartDashboard.getEntry("Angle" + index);
    nt_desired = SmartDashboard.getEntry("Desired" + index);
    nt_P = SmartDashboard.getEntry("Rotator P");

    nt_offset.setDefaultDouble(offset);
    nt_P.setDefaultDouble(0.5);
  }

  /** @return Angle without any offset correction */
  abstract public double getRawDegrees();

  /** @param voltage Voltage to motor for rotating the swerve module */
  abstract public void setVoltage(double voltage);

  /** @return Angle */
  public Rotation2d getAngle()
  {
    if (RobotBase.isSimulation())
      return Rotation2d.fromDegrees(simulated_angle);
    return Rotation2d.fromDegrees(getRawDegrees() - nt_offset.getDouble(0.0));
  }

  /** @param desired Desired angle of serve module in degrees */
  public void setAngle(double desired)
  {
    // Proportional control, with error normalized to -180..180
    double angle = getRawDegrees() - nt_offset.getDouble(0.0);
    double error = Math.IEEEremainder(desired - angle, 360.0);
    double output = error*nt_P.getDouble(0.0);
    nt_angle.setDouble(RobotBase.isReal() ? angle : desired);
    nt_desired.setDouble(desired);
    setVoltage(output);
    simulated_angle = desired;
  }
}

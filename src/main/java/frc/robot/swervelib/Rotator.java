// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Rotational part of a swerve module */
abstract public class Rotator extends SubsystemBase
{
  private final NetworkTableEntry nt_offset;
  private final NetworkTableEntry nt_P;
  private final NetworkTableEntry nt_D;
  private final NetworkTableEntry nt_clamp;
  private final NetworkTableEntry nt_angle;
  private final NetworkTableEntry nt_desired;
  private final PIDController pid = new PIDController(0,0,0);
  // TODO Use profiled PID
  // private final ProfiledPIDController pid = new ProfiledPIDController(0,0,0,
  //                     new TrapezoidProfile.Constraints(180, 180));
  private boolean initialized = false;
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
    nt_D = SmartDashboard.getEntry("Rotator D");
    nt_clamp = SmartDashboard.getEntry("Rotator Calmp");

    pid.enableContinuousInput(-180, 180);

    nt_offset.setDefaultDouble(offset);
    nt_P.setDefaultDouble(0.4);
    nt_D.setDefaultDouble(0.0);
    nt_clamp.setDefaultDouble(5.0);
  }

  /** @param brake Enable brake (if supported by motor) */
  abstract public void brake(boolean brake);

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
    if (! initialized)
    {
      // TODO Use profiled PID
      // pid.reset(getAngle().getDegrees());
      initialized = true;
    }

    // PID control, with error normalized to -180..180
    double angle = getRawDegrees() - nt_offset.getDouble(0.0);
    // double error = Math.IEEEremainder(desired - angle, 360.0);
    // double output = error*nt_P.getDouble(0.0);

    pid.setPID(nt_P.getDouble(0.0), 0.0, nt_D.getDouble(0.0));
    double output = pid.calculate(angle, desired);
    double clamp = nt_clamp.getDouble(0.0);
    output = MathUtil.clamp(output, -clamp, clamp);
    setVoltage(output);
    nt_desired.setDouble(desired);
    simulated_angle = desired;
  }
  
  @Override
  public void periodic()
  {
    nt_angle.setDouble(getAngle().getDegrees());
  }
}

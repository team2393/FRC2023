// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.parts.RotationEncoder;
import frc.robot.parts.SparkMini;

/** Rotational part of a swerve module */
public class Rotator 
{
  private final NetworkTableEntry nt_offset;
  private final NetworkTableEntry nt_P;
  private final NetworkTableEntry nt_angle;
  private final NetworkTableEntry nt_desired;

  private SparkMini motor;
  private RotationEncoder encoder;
  private double simulated_angle = 0.0;

  /** Construct rotator
   *  @param channel PMW channel 0-4
   *  @param offset Offset from 'forward' in degrees
   */
  public Rotator (int channel, double offset)
  {
    motor = new SparkMini(channel);
    encoder = new RotationEncoder(channel, offset);

    nt_offset = SmartDashboard.getEntry("Offset" + channel);
    nt_offset.setDefaultDouble(offset);

    nt_P = SmartDashboard.getEntry("P");
    nt_P.setDefaultDouble(0.05);

    nt_angle = SmartDashboard.getEntry("Angle" + channel);
    nt_desired = SmartDashboard.getEntry("Desired" + channel);
  }

  /** @param speed Speed -1..1 for rotating the swerve module */
  public void run(double speed)
  {
    motor.set(speed);
    nt_angle.setDouble(getAngle().getDegrees());
  }

  /** @param desired Desired angle of serve module in degrees */
  public void setAngle(double desired)
  {
    encoder.setZero(nt_offset.getDouble(0.0));
    // Proportional control, with error normalized to -180..180
    double angle = encoder.getHeading().getDegrees();
    double error = Math.IEEEremainder(desired - angle, 360.0);
    double output = error*nt_P.getDouble(0.0);
    nt_angle.setDouble(RobotBase.isReal() ? angle : desired);
    nt_desired.setDouble(desired);
    motor.set(output);
    simulated_angle = desired;
  }

  /** @return Angle */
  public Rotation2d getAngle()
  {
    if (RobotBase.isSimulation())
      return Rotation2d.fromDegrees(simulated_angle);
    return encoder.getHeading();
  }
}

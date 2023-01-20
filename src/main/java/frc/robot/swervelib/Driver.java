package frc.robot.swervelib;
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Forward/backwards part of swerve module
 *
 *  Controls motor voltage using feed-forward
 *  based on static voltage (ks) and velocity gain (ks),
 *  plus proportional correction
 */
abstract public class Driver extends SubsystemBase
{
  private final NetworkTableEntry nt_position;
  private final NetworkTableEntry nt_speed;
  private final NetworkTableEntry nt_ks;
  private final NetworkTableEntry nt_kv;
  private final NetworkTableEntry nt_P;
  private double zero_position = 0.0;
  private double simulated_speed = 0.0;
  private double simulated_position = 0.0;

  /** Construct Driver
   *  @param index Driver index 0..3
   *  @param ks Static voltage
   *  @param kv Velocity gain
   *  @param P Proportional gain
   */
  public Driver(int index, double ks, double kv, double P)
  {
    nt_position = SmartDashboard.getEntry("Position" + index);
    nt_speed = SmartDashboard.getEntry("Speed" + index);
    nt_ks = SmartDashboard.getEntry("Driver ks");
    nt_kv = SmartDashboard.getEntry("Driver kv");
    nt_P = SmartDashboard.getEntry("Driver P");

    nt_ks.setDefaultDouble(ks);
    nt_kv.setDefaultDouble(kv);
    nt_P.setDefaultDouble(P);
  }

  /** Reset position to zero */
  public void resetPosition()
  {
    zero_position = getRawPosition();
    simulated_position = 0.0;
  }

  /** @return Get position in meters (without zero offset) */
  abstract protected double getRawPosition();

  /** @return Get speed in meters/sec (won't be called in simulation) */
  abstract protected double getRealSpeed();

  /** @return Get speed in meters/sec */
  public double getSpeed()
  {
    if (RobotBase.isSimulation())
      return simulated_speed;
    return getRealSpeed();
  }

  /** @param voltage Voltage to motor for rotating the swerve module */
  abstract public void setVoltage(double voltage);

  /** @return Get position in meters from last 'reset' */
  public double getPosition()
  {
    if (RobotBase.isSimulation())
      return simulated_position;
    return getRawPosition() - zero_position;
  }
  
  /** @param desired_speed Speed in m/s */
  public void setSpeed(double desired_speed)
  {
    double feed_forward = nt_ks.getDouble(0.0) * Math.signum(desired_speed) + nt_kv.getDouble(0.0) * desired_speed;
    double error = desired_speed - getSpeed();
    double prop_correction = nt_P.getDouble(0.0) * error;
    setVoltage(feed_forward + prop_correction);

    // Update simulation, assume being called each period
    simulated_speed = desired_speed;
    simulated_position += desired_speed * TimedRobot.kDefaultPeriod;
  }

  @Override
  public void periodic()
  {
    nt_position.setNumber(getPosition());
    nt_speed.setNumber(getSpeed());
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Forward/backwards part of swerve module */
abstract public class Driver
{
  private final NetworkTableEntry nt_position;
  private final NetworkTableEntry nt_speed;
  private final NetworkTableEntry nt_F;
  private double zero_position = 0.0;
  private double simulated_speed = 0.0;
  private double simulated_position = 0.0;

  /** Construct Driver
   *  @param index Driver index 0..3
   */
  public Driver(int index)
  {
    nt_position = SmartDashboard.getEntry("Position" + index);
    nt_speed = SmartDashboard.getEntry("Speed" + index);
    nt_F = SmartDashboard.getEntry("Driver F");

    // Trial: About 2.8 V per m/s
    nt_F.setDefaultDouble(2.8);
  }

  /** Reset position to zero */
  public void resetPosition()
  {
    zero_position = getPosition();
    simulated_position = 0.0;
  }

  /** @return Get position in meters without zero offset */
  abstract public double getRawPosition();

  /** @return Get speed in meters/sec */
  abstract public double getSpeed();

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
    setVoltage(desired_speed * nt_F.getDouble(0));
    nt_speed.setDouble(getSpeed());
    // Update simulation, assume being called each period
    simulated_speed = desired_speed;
    simulated_position += desired_speed * TimedRobot.kDefaultPeriod;
  }
}

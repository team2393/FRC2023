// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivetrain.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Forward/backwards part of swerve module */
public class Driver 
{
  /** Maximum possible speed [m/s] */
  final public static double MAX_SPEED = 5.6;

  // Measured by driving some distance and computing
  // raw encoder counts over measured distance
  final static double COUNTS_PER_INCH = 949;
  final static double INCHES_PER_METER = 39.37;

  private final NetworkTableEntry nt_F;
  private final NetworkTableEntry nt_speed;
  private final NetworkTableEntry nt_desired;
  private final NetworkTableEntry nt_position;

  private WPI_TalonFX driver;

  private double zero_position = 0.0;

  private double simulated_speed = 0.0;
  private double simulated_position = 0.0;
  
  /** @param channel CAN bus ID 1-4 */
  public Driver (int channel)
  {
    driver = new WPI_TalonFX(channel);
    driver.setSelectedSensorPosition(0);

    nt_F = SmartDashboard.getEntry("F");
    // Trial: About 2.8 V per m/s
    nt_F.setDefaultDouble(2.8);

    nt_speed = SmartDashboard.getEntry("Speed" + channel);
    nt_desired = SmartDashboard.getEntry("Desired Speed" + channel);
    nt_position = SmartDashboard.getEntry("Position" + channel);
  }
  
  /** Reset position to zero */
  public void resetPosition()
  {
    zero_position = driver.getSelectedSensorPosition();
    simulated_position = 0.0;
  }

  /** @return Get position in meters */
  public double getPosition()
  {
    if (RobotBase.isSimulation())
      return simulated_position;
    // return Units.inchesToMeters(driver.getSelectedSensorPosition() / COUNTS_PER_INCH);
    return (driver.getSelectedSensorPosition() - zero_position) / COUNTS_PER_INCH / INCHES_PER_METER;
  }
    
  /** @return Get speed in meters/sec */
  public double getSpeed()
  {
    if (RobotBase.isReal())
      return simulated_speed;
    return driver.getSelectedSensorVelocity()*10 / COUNTS_PER_INCH / INCHES_PER_METER;
  }
  
  /** @param speed Speed -1..1 */
  public void run(double speed)
  {
    driver.set(speed);
    nt_position.setDouble(getPosition());
    nt_speed.setDouble(getSpeed());
  }

  /** @param speed Speed in m/s */
  public void setSpeed(double desired_speed)
  {
    driver.setVoltage(desired_speed * nt_F.getDouble(0));
    nt_speed.setDouble(getSpeed());
    nt_desired.setDouble(desired_speed);
    // Update simulation, assume being called each period
    simulated_speed = desired_speed;
    simulated_position += desired_speed * TimedRobot.kDefaultPeriod;
  }
}

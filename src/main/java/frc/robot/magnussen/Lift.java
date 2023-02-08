// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Lift that moves arm and grabber up/down */
public class Lift extends SubsystemBase
{
  /** Height encoder calibration */
  private static final int TICKS_PER_METER = 1;

  /** TODO Configure suitable voltage for pre-homing, slowly "up" */
  private static final double PRE_HOMING_VOLTAGE = 0.2;

  /** TODO Configure suitable voltage for homing, very slowly "down" */
  private static final double HOMING_VOLTAGE = -0.1;

  /** Motor controller with mag encoder */
  private CANSparkMax primary_motor = new CANSparkMax(RobotMap.LIFT1_ID, MotorType.kBrushless);
  
  /** Other motor */
  private CANSparkMax secondary_motor = new CANSparkMax(RobotMap.LIFT2_ID, MotorType.kBrushless);

  /** At-bottom switch */
  private DigitalInput at_bottom = new DigitalInput(RobotMap.LIFT_BOTTOM);

  private double bottom_offset = 0.0;

  public Lift()
  {
    // Primary motor has sensor and is the one we control
    primary_motor.restoreFactoryDefaults();
    primary_motor.setIdleMode(IdleMode.kBrake);

    // Secondary motor set to follow the primary
    secondary_motor.restoreFactoryDefaults();
    secondary_motor.setIdleMode(IdleMode.kBrake);

    // Motors need to run in opposite direction,
    // so _one_(!) of them must be inverted    
    secondary_motor.setInverted(true);

    // We commmand the primary, secondary follows
    secondary_motor.follow(primary_motor);

    // TODO SmartDashboard.getEntry(..
    SmartDashboard.setDefaultNumber("Lift kg", 0.0);
    SmartDashboard.setDefaultNumber("Lift ks", 0.0);
    SmartDashboard.setDefaultNumber("Lift P", 0.0);
  }

  /** @return Is lift at bottom? */
  public boolean atBottom()
  {
    // Must be 'true' when at bottom
    boolean is_at_bottom = at_bottom.get();

    // Reset encoder's zero offset?
    if (is_at_bottom)
      bottom_offset = primary_motor.getEncoder().getPosition();

    return is_at_bottom;
  }

  /** Move lift up until it clears the bottom switch
   *  @return Has lift been pre-homed?
   */
  public boolean pre_home()
  {
    if (atBottom())
    { // Move up, not pre-homed
      setVoltage(PRE_HOMING_VOLTAGE);
      return false;
    }
    // else: Cleared the switch, done
    setVoltage(0);
    return true;
  }

  /** Move lift down until it hits the bottom switch
   *  @return Has lift been homed?
   */
  public boolean home()
  {
    if (atBottom())
    {
      setVoltage(0);
      return true;
    }
    // else
    setVoltage(HOMING_VOLTAGE);
    return false;
  }

  /** @return Lift height in meters */
  public double getHeight()
  {
    return (primary_motor.getEncoder().getPosition() - bottom_offset) / TICKS_PER_METER;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putBoolean("At Bottom", atBottom());
    SmartDashboard.putNumber("Height", getHeight());
  }

  /** @param voltage Lift voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    // Don't go below the bottom!
    if (atBottom()  &&  voltage < 0)
      voltage = 0.0;

    primary_motor.setVoltage(voltage);
    SmartDashboard.putNumber("Lift Voltage", voltage);
  }

  public void setHeight(double desired_height)
  {
    // Compare w/ ElevatorFeedforward
    // Gravity gain, always applied to counteract gravity
    // unless at bottom
    double kg = SmartDashboard.getNumber("Lift kg", 0.0);
    // Static gain, minimum voltage to get moving
    double ks = SmartDashboard.getNumber("Lift ks", 0.0);
    // Propotional gain to correct height error
    double P  = SmartDashboard.getNumber("Lift P", 0.0);

    double error = desired_height - getHeight();
    double voltage;

    // Don't run motor below the bottom position!
    // Are we at the bottom, and
    // a) We want to be at zero height or below?
    // b) We want to move 'down' (which we can't)?
    if (atBottom() &&
        (desired_height <= 0.0  ||  desired_height < getHeight()))
      voltage = 0.0;
    else
      voltage = kg  +  ks * Math.signum(error)  +  P * error;
    setVoltage(voltage);
  }
}

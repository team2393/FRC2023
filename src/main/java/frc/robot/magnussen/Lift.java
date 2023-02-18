// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Lift that moves arm and grabber up/down
 * 
 *  When unpowered, lift settles down at bottom because of its weight.
 *  We read that zero position from the encoder when first enabled.
 *  During tests, first enablement is likely teleop.
 *  In competition, first enablement is auto, and by the time
 *  we enable teleop the lift may already have moved up.
 *  Using first enablement after bootup should cover both cases.
 * 
 *  Move to requested height using feed forward and PID,
 *  except for moving to "zero" which moves to a certain height above zero
 *  and then simply lets lift settle by unpowering the motors. 
 */
public class Lift extends SubsystemBase
{
  /** Height encoder calibration */
  private static final double REVS_PER_METER = 1;

  /** Maximum permitted height */
  private static final double MAX_HEIGHT = 1.5;

  /** Height below which we let the lift settle on its own */
  private static final double SETTLE_THRESHOLD = 0.1;

  /** Motor controller with encoder */
  private CANSparkMax primary_motor = new CANSparkMax(RobotMap.LIFT1_ID, MotorType.kBrushless);
  
  /** Other motor */
  private CANSparkMax secondary_motor = new CANSparkMax(RobotMap.LIFT2_ID, MotorType.kBrushless);

  /** Has position be calibrated? */
  private boolean calibrated = false;

  /** Raw encoder reading at bottom position */
  private double bottom_offset = 0.0;

  /** Network table entries */
  private NetworkTableEntry nt_height, nt_kg, nt_ks;

  /** PID */
  private PIDController pid = new PIDController(0, 0, 0);

  private double simulated_height = 0.0;

  public Lift()
  {
    // Primary motor is the one we control
    primary_motor.restoreFactoryDefaults();
    primary_motor.clearFaults();
    primary_motor.setSmartCurrentLimit(20);
    primary_motor.setIdleMode(IdleMode.kCoast);

    // Secondary motor set to follow the primary
    secondary_motor.restoreFactoryDefaults();
    secondary_motor.clearFaults();
    secondary_motor.setSmartCurrentLimit(20);
    secondary_motor.setIdleMode(IdleMode.kCoast);

    // Motors need to run in opposite direction,
    // so _one_(!) of them must be inverted    
    secondary_motor.setInverted(true);

    // We commmand the primary motor, let secondary follow
    secondary_motor.follow(primary_motor);

    nt_height = SmartDashboard.getEntry("Lift Height");
    nt_kg = SmartDashboard.getEntry("Lift kg");
    nt_ks = SmartDashboard.getEntry("Lift ks");
    nt_kg.setDefaultDouble(0.0);
    nt_ks.setDefaultDouble(0.0);
    SmartDashboard.putData("Lift PID", pid);
  }

  @Override
  public void periodic()
  {
    // Enabled for the first time, never calibrated?
    if (! calibrated  &&  RobotState.isEnabled())
    {
      // Reset encoder zero/bottom position
      bottom_offset = primary_motor.getEncoder().getPosition();
      calibrated = true;
      System.err.println("Calibrated lift bottom at " + bottom_offset + " revs");
    }
    nt_height.setDouble(getHeight());
  }  

  /** @return Lift height in meters */
  public double getHeight()
  {
    if (RobotBase.isSimulation())
      return simulated_height;
    return (primary_motor.getEncoder().getPosition() - bottom_offset) / REVS_PER_METER;
  }

  /** @param voltage Lift voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    primary_motor.setVoltage(voltage);
  }

  public void setHeight(double desired_height)
  {
    if (RobotBase.isSimulation())
    {
      simulated_height = desired_height;
      return;
    }

    double height = getHeight();

    // Don't run above top position
    if (desired_height >= MAX_HEIGHT)
      desired_height = MAX_HEIGHT;

    // Trying to move to bottom?
    if (desired_height <= SETTLE_THRESHOLD)
    {
      // Don't "run" into the bottom.
      // Move to lower threshold
      desired_height = SETTLE_THRESHOLD;
      // If we are within 5 cm, simply let lift settle
      if (height <= SETTLE_THRESHOLD + 0.05)
      {
        primary_motor.setIdleMode(IdleMode.kCoast);
        secondary_motor.setIdleMode(IdleMode.kCoast);
        setVoltage(0.0);
        return;
      }
      // else: Move down to SETTLE_THRESHOLD..
    }

    // Not settling to bottom but actively moving to position, so enable brake
    primary_motor.setIdleMode(IdleMode.kBrake);
    secondary_motor.setIdleMode(IdleMode.kBrake);

    // Compare w/ ElevatorFeedforward
    // kg  - Gravity gain, always applied to counteract gravity
    // ks  - Static gain, minimum voltage to get moving
    // PID - .. to correct height error
    double error = desired_height - height;
    double voltage = nt_kg.getDouble(0.0)
                   + nt_ks.getDouble(0.0) * Math.signum(error)
                   + pid.calculate(height, desired_height);
    setVoltage(voltage);
  }
}

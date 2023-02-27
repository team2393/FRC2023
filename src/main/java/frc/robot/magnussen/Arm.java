// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm that rotates and extends/contracts */
public class Arm extends SubsystemBase
{
  /** Arm length [m] */
  public static final double LENGTH_EXTENDED = 0.74, LENGTH_RETRACTED = 0.58;

  /** Motor controller */
  private final CANSparkMax motor = new CANSparkMax(RobotMap.ARM_ID, MotorType.kBrushless);

  /** Through Bore Encoder to measure angle.
   *  'A'/'S' switch on side of encoder must be in 'A' position.
   *  Absolute readout can use (white, red, black) into DI,
   *  but we have it plugged into the motor's SparkMax
   */
  private final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

  private final Solenoid extender = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.ARM_EXTENDER);

  private double simulated_angle = -150.0;
  
  private NetworkTableEntry nt_angle;
  private NetworkTableEntry nt_extended;
  private NetworkTableEntry nt_offset;
  private NetworkTableEntry nt_kg_in;
  private NetworkTableEntry nt_kg_out;
  private NetworkTableEntry nt_ks;
  // PID for angle, not adjusting faster than 90 deg/sec (same accel.)
  private ProfiledPIDController pid = new ProfiledPIDController(0.15, 0, 0,
                                          new TrapezoidProfile.Constraints(90, 90));

  public Arm()
  {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    // Positive voltage moves to positive angles, "up"
    motor.setInverted(true);
    motor.setSmartCurrentLimit(20);

    extender.set(false);

    nt_angle = SmartDashboard.getEntry("Arm Angle");
    nt_extended = SmartDashboard.getEntry("Arm Extended");
    nt_offset = SmartDashboard.getEntry("Arm Offset");
    nt_offset.setDefaultDouble(91.25);
    nt_kg_in = SmartDashboard.getEntry("Arm kg in");
    nt_kg_in.setDefaultDouble(0.3);
    nt_kg_out = SmartDashboard.getEntry("Arm kg out");
    nt_kg_out.setDefaultDouble(0.3);
    nt_ks = SmartDashboard.getEntry("Arm ks");
    nt_ks.setDefaultDouble(0.07);
    
    SmartDashboard.putData("Arm PID", pid);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.reset(getAngle());
  }

  /** @return Arm angle in degrees, zero = horizontal, -90 = vertical down */
  public double getAngle()
  {
    if (RobotBase.isSimulation())
      return simulated_angle;
    // Change 'turns' into degrees,
    // fix offset, bracked to -180..+180
    return Math.IEEEremainder(encoder.getPosition()*360 - nt_offset.getDouble(0.0), 360);
  }

  @Override
  public void periodic()
  {
    nt_angle.setNumber(getAngle());
    nt_extended.setBoolean(extender.get());
  }

  /** @param voltage Arm voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }

  /** @return Is arm extended? */
  public boolean isExtended()
  {
    return extender.get();
  }

  /** @param out Extend arm out, or pull in? */
  public void extend(boolean out)
  {
    extender.set(out);
  }

  public void setAngle(double desired_angle)
  {
    if (RobotBase.isSimulation())
    {
      final double adjust = 0.5;
      if (desired_angle > simulated_angle + adjust)
        simulated_angle += adjust;
      else if (desired_angle < simulated_angle - adjust)
        simulated_angle -= adjust;
      else
        simulated_angle = desired_angle;
      return;
    }
    // Compare w/ ArmFeedforward
    // Gravity gain, always applied to counteract gravity,
    // but different for extension in/out
    double kg;
    if (extender.get())
      kg = nt_kg_out.getDouble(0.0);
    else
      kg = nt_kg_in.getDouble(0.0);
    // Static gain, minimum voltage to get moving
    double ks = nt_ks.getDouble(0.0);

    // If arm is horizontal, cos(0) = 1 --> Apply full kg
    // If arm is down, cos(-90 deg) = 0 --> No kg
    double current_angle = getAngle();
    double error = Math.IEEEremainder(desired_angle - current_angle, 360.0);
    double voltage = ks * Math.signum(error) 
                   + kg * Math.cos(Math.toRadians(current_angle))
                   + pid.calculate(current_angle, desired_angle);
    setVoltage(voltage);
  }

/* Example for handling the -180..180 range
  public static void main(String[] args)
  {
    double setpoint, readback, error;
    
    setpoint = -45;
    readback = -30;
    error = setpoint - readback;
    System.out.println("Set " + setpoint + ", readb " + readback + " -> error " + error);

    setpoint = -180;
    readback = -175;
    error = setpoint - readback;
    System.out.println("Set " + setpoint + ", readb " + readback + " -> error " + error);

    setpoint = -180;
    readback = +175;
    error = setpoint - readback;
    System.out.println("Set " + setpoint + ", readb " + readback + " -> error " + error);

    error = Math.IEEEremainder(setpoint - readback, 360);
    System.out.println("Set " + setpoint + ", readb " + readback + " -> error " + error);

    setpoint = 0;
    readback = -5;
    error = Math.IEEEremainder(setpoint - readback, 360);
    System.out.println("Set " + setpoint + ", readb " + readback + " -> error " + error);

    setpoint = 0;
    readback = 5;
    error = Math.IEEEremainder(setpoint - readback, 360);
    System.out.println("Set " + setpoint + ", readb " + readback + " -> error " + error);
  }
*/
}

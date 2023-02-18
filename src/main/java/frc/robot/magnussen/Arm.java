// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm that rotates and extends/contracts */
public class Arm extends SubsystemBase
{
  /** Motor controller */
  private final CANSparkMax motor = new CANSparkMax(RobotMap.ARM_ID, MotorType.kBrushless);

  /** Through Bore Encoder to measure angle.
   *  'A'/'S' switch on side of encoder must be in 'A' position.
   *  Absolute readout can use (white, red, black) into DI,
   *  but we have it plugged into the motor's SparkMax
   */
  private final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

  private final Solenoid extender = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.ARM_EXTENDER);

  private double simulated_angle = -90.0;

  public Arm()
  {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    // Positive voltage moves to positive angles, "up"
    motor.setInverted(true);
    motor.setSmartCurrentLimit(20); // TODO current limit?

    extender.set(false);

    // TODO SmartDashboard.getEntry(..
    SmartDashboard.setDefaultNumber("Arm Offset", 91.25);
    SmartDashboard.setDefaultNumber("Arm kg in", 0.3);
    SmartDashboard.setDefaultNumber("Arm kg out", 0.3);
    SmartDashboard.setDefaultNumber("Arm ks", 0.07);
    SmartDashboard.setDefaultNumber("Arm P", 0.15);
  }

  /** @return Arm angle in degrees, zero = horizontal, -90 = vertical down */
  public double getAngle()
  {
    if (RobotBase.isSimulation())
      return simulated_angle;
    // Change 'turns' into degrees,
    // fix offset, bracked to -180..+180
    return Math.IEEEremainder(encoder.getPosition()*360 - SmartDashboard.getNumber("Arm Offset", 0.0), 360);
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putBoolean("Arm Extended", extender.get());
  }

  /** @param voltage Arm voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
    SmartDashboard.putNumber("Arm Voltage", voltage);
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
      simulated_angle = desired_angle;
      return;
    }
    // Compare w/ ArmFeedforward
    // Gravity gain, always applied to counteract gravity,
    // but different for extension in/out
    double kg;
    if (extender.get())
      kg = SmartDashboard.getNumber("Arm kg out", 0.0);
    else
      kg = SmartDashboard.getNumber("Arm kg in", 0.0);
    // Static gain, minimum voltage to get moving
    double ks = SmartDashboard.getNumber("Arm ks", 0.0);
    // Propotional gain to correct angle error
    double P  = SmartDashboard.getNumber("Arm P", 0.0);

    // If arm is horizontal, cos(0) = 1 --> Apply full kg
    // If arm is down, cos(-90 deg) = 0 --> No kg
    double current_angle = getAngle();
    double error = Math.IEEEremainder(desired_angle - current_angle, 360.0);
    double voltage = ks * Math.signum(error) 
                   + kg * Math.cos(Math.toRadians(current_angle))
                   +  P * error;
    setVoltage(voltage);
  }
}

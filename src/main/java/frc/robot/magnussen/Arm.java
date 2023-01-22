// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm that rotates and extends/contracts */
public class Arm extends SubsystemBase
{
  /** Motor controller */
  private WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.ARM_ID);

  /** Through Bore Encoder to measure angle
   *  Absolute readout uses (white, red, black) into DI
   */
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.ARM_ANGLE));

  private Solenoid extender = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.ARM_EXTENDER);

  public Arm()
  {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);

    encoder.reset();

    extender.set(false);

    // TODO SmartDashboard.getEntry(..
    SmartDashboard.setDefaultNumber("Arm Offset", 0.0);
    SmartDashboard.setDefaultNumber("Arm kg in", 0.0);
    SmartDashboard.setDefaultNumber("Arm kg out", 0.0);
    SmartDashboard.setDefaultNumber("Arm ks", 0.0);
    SmartDashboard.setDefaultNumber("Arm P", 0.0);
  }

  /** @return Arm angle in degrees, zero = horizontal, -90 = vertical down */
  public double getAngle()
  {
    return encoder.getAbsolutePosition() - SmartDashboard.getNumber("Arm Offset", 0.0);
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

  /** @param out Extend arm out, or pull in? */
  public void extend(boolean out)
  {
    extender.set(out);
  }

  public void setAngle(double desired_angle)
  {
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
    double error = desired_angle - current_angle;
    double voltage = ks * Math.signum(error) 
                   + kg * Math.cos(Math.toRadians(current_angle))
                   +  P * error;
    setVoltage(voltage);
  }
}

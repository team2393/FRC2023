// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
  private CANSparkMax motor = new CANSparkMax(RobotMap.INTAKE_ANGLE, MotorType.kBrushless);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.INTAKE_ANGLE));

  public Intake()
  {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    encoder.reset();

    // TODO SmartDashboard.getEntry(..
    SmartDashboard.setDefaultNumber("Intake Offset", 0.0);
    SmartDashboard.setDefaultNumber("Intake Voltage", 0.0);
  }

  /** @return Intake angle in degrees */
  public double getAngle()
  {
    return encoder.getAbsolutePosition() - SmartDashboard.getNumber("Intake Offset", 0.0);
  }

  /** @param voltage Arm voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);
  }

  /** @param angle Intake angle, positive for "up" */
  public void setAngle(double desired_angle)
  {
    // Static gain, minimum voltage to get moving
    double ks = SmartDashboard.getNumber("Arm ks", 0.0);
    // Propotional gain to correct angle error
    double P  = SmartDashboard.getNumber("Arm P", 0.0);

    // If arm is horizontal, cos(0) = 1 --> Apply full kg
    // If arm is down, cos(-90 deg) = 0 --> No kg
    double current_angle = getAngle();
    double error = desired_angle - current_angle;
    double voltage = ks * Math.signum(error) 
                   + Math.cos(Math.toRadians(current_angle))
                   + P * error;
    setVoltage(voltage);
  }
}

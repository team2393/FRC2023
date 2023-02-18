// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
  private CANSparkMax motor = new CANSparkMax(RobotMap.INTAKE_ANGLE, MotorType.kBrushless);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.INTAKE_ANGLE));
  private double simulated_angle = 90.0;

  public Intake()
  {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    encoder.reset();

    // TODO SmartDashboard.getEntry(..
    SmartDashboard.setDefaultNumber("Intake Offset", 0.0);
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake Angle", getAngle());
  }

  /** @return Intake angle in degrees */
  public double getAngle()
  {
    if (RobotBase.isSimulation())
      return simulated_angle;
    return encoder.getAbsolutePosition() - SmartDashboard.getNumber("Intake Offset", 0.0);
  }

  /** @param voltage Intake voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);
  }

  /** @param angle Intake angle, positive for "up" */
  public void setAngle(double desired_angle)
  {
    if (RobotBase.isSimulation())
    {
      simulated_angle = desired_angle;
      return;
    }
    // Gravity gain, always applied to counteract gravity,
    // but different for extension in/out
    double kg;
    // TODO if (extender.get())
    //   kg = SmartDashboard.getNumber("Intake kg out", 0.0);
    // else
      kg = SmartDashboard.getNumber("Intake kg in", 0.0);
    // Static gain, minimum voltage to get moving
    double ks = SmartDashboard.getNumber("Intake ks", 0.0);
    // Propotional gain to correct angle error
    double P  = SmartDashboard.getNumber("Intake P", 0.0);

    // If intake is horizontal, cos(0) = 1 --> Apply full kg
    // If intake is down, cos(-90 deg) = 0 --> No kg
    double current_angle = getAngle();
    double error = desired_angle - current_angle;
    double voltage = ks * Math.signum(error) 
                   + kg * Math.cos(Math.toRadians(current_angle))
                   +  P * error;
    setVoltage(voltage);
  }
}

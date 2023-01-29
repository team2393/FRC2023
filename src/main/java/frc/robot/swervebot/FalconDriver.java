// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervebot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.swervelib.Driver;

/** Driver using Falcon */
public class FalconDriver extends Driver
{
  private final static int TICKS_PER_METER = 42962;

  private final WPI_TalonFX motor;

  public FalconDriver(int index)
  {
    super(index, 0.39, 2.32, 3);
    motor = new WPI_TalonFX(index + 1);
    motor.configFactoryDefault();
    motor.configOpenloopRamp(0.3);
  }
  
  @Override
  public void brake(boolean brake)
  {
    motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  protected double getRawPosition()
  {
    return motor.getSelectedSensorPosition() / TICKS_PER_METER;
  }

  protected double getRealSpeed()
  {
    // Convert speed in ticks per 0.1 second to m/s
    return motor.getSelectedSensorVelocity() * 10.0 / TICKS_PER_METER;
  }

  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.swervelib.Driver;

/** Driver using Falcon */
public class FalconDriver extends Driver
{
  // TODO
  // (41983+47258+45040+40705)/4
  private final static int TICKS_PER_METER = 43746;

  private final WPI_TalonFX motor;

  public FalconDriver(int index)
  {
    // TODO ks, kv, P gains
    super(index, 0.6, 2.15, 1);
    motor = new WPI_TalonFX(RobotMap.DRIVER_ID[index], RobotMap.CANIVORE);
    motor.configFactoryDefault();
    motor.configOpenloopRamp(0.3);

    // Enable limit to 18 amp if exceeding 20 amp for 0.2 sec
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 18, 20, 0.2));
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

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.swervelib.Rotator;

public class FC_Rotator extends Rotator
{
  private final WPI_TalonFX motor;
  private final CANCoder sensor;

  public FC_Rotator(int index, double offset)
  {
    super(index, offset, 0.3, 0.1, 0.0, 0.0002, 3);

    motor = new WPI_TalonFX(RobotMap.ROTATOR_ID[index], RobotMap.CANIVORE);
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(true);
    
    sensor = new CANCoder(RobotMap.ANGLE_ID[index], RobotMap.CANIVORE);
    sensor.configFactoryDefault();
    sensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);    
  }
    
  @Override
  public void brake(boolean brake)
  {
    motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public double getRawDegrees() 
  {
    return sensor.getAbsolutePosition();
  }

  @Override
  public void setVoltage(double voltage) 
  {
    motor.setVoltage(voltage);
  }
}

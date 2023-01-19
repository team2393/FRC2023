// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.swervebot;

import frc.robot.parts.RotationEncoder;
import frc.robot.parts.SparkMini;
import frc.robot.swervelib.Rotator;

/** Rotator using SparkMini and absolute encoder */
public class SparkMiniRotator extends Rotator
{
  private final SparkMini motor;
  private final RotationEncoder encoder;

  /** Construct Rotator
   *  @param index Rotator index 0..3
   *  @param offset Offset from 'forward' in degrees
   */
  public SparkMiniRotator(int index, double offset)
  {
    super(index, offset);
    motor = new SparkMini(index);
    encoder = new RotationEncoder(index, 0.0);
  }

  public double getRawDegrees()
  {
    return encoder.getRawHeading();
  }

  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }
}

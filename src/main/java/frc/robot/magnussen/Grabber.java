// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Grabber */
public class Grabber extends SubsystemBase
{
  // TODO Pick suitable voltages
  public static final double INTAKE_VOLTAGE = 2.0;
  public static final double RELEASE_VOLTAGE = -1.5;

  /** Motor controller */
  private CANSparkMax spinner = new CANSparkMax(RobotMap.GRABBER_ID, MotorType.kBrushless);

  // TODO Proximity sensor to stop when game piece is in grabber?
  // TODO Methods pullIn() until something's captures,
  //      pushOut() for a few seconds?

  public Grabber()
  {
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kCoast);
   spinner.setSmartCurrentLimit(30); // TODO current limit?
  }

  /** @param voltage Spinner voltage, positive for 'in' */
  public void setVoltage(double voltage)
  {
    spinner.setVoltage(voltage);
  }
}

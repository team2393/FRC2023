// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Grabber */
public class Grabber extends SubsystemBase
{
  // TODO Pick suitable voltages
  public static final double INTAKE_VOLTAGE = 2.0;
  public static final double RELEASE_VOLTAGE = -1.5;

  /** Motor controller */
  private WPI_TalonSRX spinner = new WPI_TalonSRX(RobotMap.SPINNER_ID);

  // TODO Proximity sensor to stop when game piece is in grabber?
  // TODO Methods pullIn() until something's captures,
  //      pushOut() for a few seconds?

  public Grabber()
  {
    spinner.configFactoryDefault();
    spinner.setNeutralMode(NeutralMode.Coast);
    spinner.configOpenloopRamp(0.5);
  }

  /** @param voltage Spinner voltage, positive for 'in' */
  public void setVoltage(double voltage)
  {
    spinner.setVoltage(voltage);
  }
}

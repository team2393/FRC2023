// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Grabber */
public class Grabber extends SubsystemBase
{
  public static final double CUBE_VOLTAGE = 8;
  public static final double CONE_VOLTAGE = 8.0;
  public static final double EJECT_VOLTAGE = -10.0;

  private DigitalInput sensor = new DigitalInput(RobotMap.GRABBER_SENSOR);

  /** Motor controller */
  private CANSparkMax spinner = new CANSparkMax(RobotMap.GRABBER_ID, MotorType.kBrushless);
  private NetworkTableEntry nt_sensor;


  public Grabber()
  {
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kCoast);
    spinner.setSmartCurrentLimit(20);

    nt_sensor = SmartDashboard.getEntry("Grabber");

    setDefaultCommand(new GrabberOffCommand(this));
  }

  /** @return Do we sense a cube or cone? */
  public boolean haveGamepiece()
  {
    return sensor.get();
  }
  
  @Override
  public void periodic()
  {
    nt_sensor.setBoolean(haveGamepiece());
  }

  /** @param voltage Spinner voltage, positive for 'in' */
  public void setVoltage(double voltage)
  {
    spinner.setVoltage(voltage);
  }
}

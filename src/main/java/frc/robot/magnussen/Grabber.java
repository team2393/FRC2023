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
  // TODO Pick suitable voltages
  public static final double CUBE_VOLTAGE = 2.0;
  public static final double CONE_VOLTAGE = 5.0;
  public static final double EJECT_VOLTAGE = -2.0;

  private DigitalInput cube_sensor = new DigitalInput(RobotMap.CUBE_SENSOR);
  private DigitalInput cone_sensor = new DigitalInput(RobotMap.CONE_SENSOR);

  /** Motor controller */
  private CANSparkMax spinner = new CANSparkMax(RobotMap.GRABBER_ID, MotorType.kBrushless);
  private NetworkTableEntry nt_cube, nt_cone;


  public Grabber()
  {
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kCoast);
    spinner.setSmartCurrentLimit(30); // TODO current limit?

    nt_cube = SmartDashboard.getEntry("Cube");
    nt_cone = SmartDashboard.getEntry("Cone");

    setDefaultCommand(new GrabberOffCommand(this));
  }

  /** @return Do we sense a cube? */
  public boolean haveCube()
  {
    return cube_sensor.get();
  }

  /** @return Do we sense a cone? */
  public boolean haveCone()
  {
    return cone_sensor.get();
  }
  
  @Override
  public void periodic()
  {
    nt_cube.setBoolean(haveCube());
    nt_cone.setBoolean(haveCone());
  }

  /** @param voltage Spinner voltage, positive for 'in' */
  public void setVoltage(double voltage)
  {
    spinner.setVoltage(voltage);
  }
}

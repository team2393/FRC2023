// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Intake
 *
 *  Rotates from zero degrees for 'horizontally out'
 *  to 90 for 'up' and further to 'all in'.
 * 
 *  Spinner to capture game pieces. 
 */
public class Intake extends SubsystemBase
{
  /** Intake length [m] */
  public static final double LENGTH = 0.39;

  public static final double SPINNER_VOLTAGE = 4.0;
  private CANSparkMax rotator = new CANSparkMax(RobotMap.INTAKE_ID, MotorType.kBrushless);
  private CANSparkMax spinner = new CANSparkMax(RobotMap.INTAKE_SPINNER, MotorType.kBrushless);

  /** Through Bore Encoder on DIO
   *  'A'/'S' switch on side of encoder must be in 'A' position.
   *  Absolute readout via (white, red, black)
   */
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.INTAKE_ANGLE));
  
  // Encoder on SparkMAX?
  // private SparkMaxAbsoluteEncoder encoder = rotator.getAbsoluteEncoder(Type.kDutyCycle);

  private double simulated_angle = 90.0;

  private NetworkTableEntry nt_offset, nt_kg, nt_angle;

  private final ProfiledPIDController pid = new ProfiledPIDController(0.05, 0, 0,
                                                                      new Constraints(180.0, 90.0));

  public Intake()
  {
    rotator.restoreFactoryDefaults();
    rotator.setIdleMode(IdleMode.kCoast);
    rotator.setInverted(true);
    rotator.setSmartCurrentLimit(20);

    // Use low current limit to hold game pieces
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kCoast);
    spinner.setSmartCurrentLimit(2);

    encoder.reset();
    
    nt_offset = SmartDashboard.getEntry("Intake Offset");
    nt_offset.setDefaultDouble(-100.0);
    nt_kg = SmartDashboard.getEntry("Intake kg");
    nt_kg.setDefaultDouble(0.2);
    nt_angle = SmartDashboard.getEntry("Intake Angle");

    pid.enableContinuousInput(-180.0, 180.0);
    pid.reset(getAngle());
  }

  @Override
  public void periodic()
  {
    nt_angle.setDouble(getAngle());
  }

  /** @return Intake angle in degrees */
  public double getAngle()
  {
    if (RobotBase.isSimulation())
      return simulated_angle;
    return Math.IEEEremainder(encoder.getAbsolutePosition()*360.0 - nt_offset.getDouble(0.0),
                              360);
  }

  /** @param voltage Spinner voltage, positive for "in" */
  public void setSpinner(double voltage)
  {
    spinner.setVoltage(voltage);
  }

  /** @param voltage Intake voltage, positive for "up" */
  public void setVoltage(double voltage)
  {
    rotator.setVoltage(voltage);
  }

  /** @param angle Intake angle, zero for horizontally out, positive for "up" */
  public void setAngle(double desired_angle)
  {
    if (RobotBase.isSimulation())
    {
      simulated_angle = desired_angle;
      return;
    }
    // Gravity gain, always applied to counteract gravity
    double kg = nt_kg.getDouble(0.0);

    // If intake is horizontal, cos(0) = 1 --> Apply full kg
    // If intake is vertically up, cos(90 deg) = 0 --> No kg
    double current_angle = getAngle();
    double voltage = kg * Math.cos(Math.toRadians(current_angle))
                   + pid.calculate(current_angle, desired_angle);
    setVoltage(voltage);
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(RobotMap.INTAKE_ANGLE, MotorType.kBrushless);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.INTAKE_ANGLE));

  public Intake() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    encoder.reset();

    // TODO SmartDashboard.getEntry(..
    SmartDashboard.setDefaultNumber("Intake Offset", 0.0);
    SmartDashboard.setDefaultNumber("Intake Voltage", 0.0);
  }

  /** @return Intake angle in degrees */
  public double getAngle() {
    return encoder.getAbsolutePosition() - SmartDashboard.getNumber("Intake Offset", 0.0);
  }

  /** @param voltage Arm voltage, positive for "up" */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);
  }
}

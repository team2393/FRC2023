// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Arm test robot
 *
 *  Setup Procedure:
 * - Use correct motor and sensor types, ID, pneumatics module type, ..
 *
 * Disabled:
 * - Check if "Arm Angle" is correctly indicated,
 *   adjust "Arm Offset" so straight out horizontal reads 0 degrees,
 *   vertically down reads -90 degrees,
 *   above horizontal gives positive angle
 * 
 * Teleop, Y, A:
 * - Does arm start out retracted?
 * - Check if 'Y' extends arm and 'A' retracts arm
 *
 * Teleop, right stick:
 * - Check if moving 'up' with positive voltage moves indeed 'up'.
 *   If not, reverse motor wiring.
 * - Try to hold arm horizontally, note required voltage
 *   while in vs. out
 * 
 * Autonomouse:
 * - Carefully move arm manually and adjust "Arm kg in" and "out"
 *   such that arm counteracts gravity and stays put.
 *   (Need to move arm in/out in teleop.)
 * - Enter a setpoint above current angle
 *   adjust ks such that motor just barely starts moving 'up'
 * - Enter different setpoints and adjust P such that arm gets there
 * 
 * .. then update to PID, then ProfiledPID?
 */
public class ArmTestRobot extends CommandBaseRobot
{
  private final Pneumatics pneumatics = new Pneumatics();
  private final Arm arm = new Arm();
  private final Grabber grabber = new Grabber();

  @Override
  public void robotInit()
  {
    super.robotInit();
    OI.reset();
    SmartDashboard.setDefaultNumber("Setpoint1", 0.0);
    SmartDashboard.setDefaultNumber("Setpoint2", 0.0);
  }

  @Override
  public void teleopPeriodic()
  {
    // Extend/retract arm    
    if (OI.joystick.getAButtonPressed())
      arm.extend(true);
    if (OI.joystick.getYButtonPressed())
      arm.extend(false);
    
      // For 'up', send position voltage
    double voltage = -5.0 * MathUtil.applyDeadband(OI.joystick.getRightY(), 0.1);
    arm.setVoltage(voltage);
    SmartDashboard.putNumber("Arm Voltage", voltage);

    // 'up' to pull game piece in
    voltage = -8.0 * MathUtil.applyDeadband(OI.joystick.getLeftY(), 0.1);
    grabber.setVoltage(voltage);
    SmartDashboard.putNumber("Grabber Voltage", voltage);
  }

  @Override
  public void autonomousPeriodic()
  {
    double angle = (System.currentTimeMillis() / 5000) % 2 == 1
                 ? SmartDashboard.getNumber("Setpoint1", 0)
                 : SmartDashboard.getNumber("Setpoint2", 0);
    arm.setAngle(angle);
  }
}

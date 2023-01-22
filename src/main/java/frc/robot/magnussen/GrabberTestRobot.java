// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Grabber test robot
 *
 *  TODO Setup Procedure:
 * - Use correct motor IDs
 * 
 * Teleop, right stick:
 * - Check if moving 'up' with positive voltage moves
 *   all spinners to pull game piece 'in'.
 *   If not, reverse motor wiring.
 * 
 * - Determine suitable voltages for pulling in
 *   and pushing out
 */
public class GrabberTestRobot extends CommandBaseRobot
{
  private final Grabber grabber = new Grabber();

  @Override
  public void teleopPeriodic()
  {
      // For 'up', send position voltage
      double voltage = -5.0 * OI.joystick.getRightY();
      grabber.setVoltage(voltage);

      SmartDashboard.putNumber("Voltage", voltage);
  }
}

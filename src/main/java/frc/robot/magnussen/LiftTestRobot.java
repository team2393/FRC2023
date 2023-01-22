// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Lift test robot
 *
 *  TODO Setup Procedure:
 * - Use correct motor and sensor types, ID, ..
 *
 * Disabled:
 * - Check if "At Bottom" is correctly indicated,
 *   'true' when at bottom.abstract.
 *   Ideally, wiring could be fail-safe..
 * 
 * Teleop, right stick:
 * - Check if moving 'up' with positive voltage moves indeed 'up'.
 *   If not, reverse motor wiring.
 * - Check if moving 'down' stops when hitting bottom switch
 * - Check if indicated "Height" zeroes when hitting bottom switch
 * - Calibrate height encoder
 * 
 * Autonomouse:
 * - Start with "Setpoint" = 0.0, assert that motor is not powered
 * - Enter negative setpoint, assert that motor is not powered
 * - Disable, move lift halfway up, enter halfway Setpoint, enable auto.
 *   Adjust kg such that motor counteracts gravity and lift stays put
 * - Enter a setpoint above current height and
 *   adjust ks such that motor just barely starts moving 'up'
 * - Enter different setpoints and adjust P such that lift gets there
 * 
 * .. then update to PID, then ProfiledPID?
 */
public class LiftTestRobot extends CommandBaseRobot
{
  private final Lift lift = new Lift();

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("Setpoint", 0.0);
  }

  @Override
  public void teleopPeriodic()
  {
    // For 'up', send position voltage
    lift.setVoltage(-5.0 * OI.joystick.getRightY());
  }

  @Override
  public void autonomousPeriodic()
  {
    lift.setHeight(SmartDashboard.getNumber("Setpoint", 0.0));
  }
}

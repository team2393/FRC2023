// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Lift test robot
 *
 * Setup Procedure:
 * - Use correct motors and sensor types, ID, ..
 * 
 * Teleop, right stick:
 * - Disconnect both motors' power wires from speed controllers
 * - After bootup, briefly enable teleop. This first enablement
 *   should zero the height reading
 * - Calibrate height encoder REV_PER_METER and MAX_HEIGHT
 * - Connect power wires for primary motor
 * - Enable teleop. Check if moving 'up' with positive voltage
 *   indeed moves primary motor 'up'.
 *   If not, switch inversion of both motors.
 * - Connect secondary motor to speed controller and check that it moves
 *   in correct direction with primary. If not, update its inversion setting.
 * - Moving lift up and down, check that it stops at bottom (zero) and MAX_HEIGHT.
 * 
 * Auto-no-mouse:
 * - Start with "Setpoint" = 0.0, assert that motor is not powered
 * - Enter negative setpoint, assert that motor is not powered
 * - Disable, move lift halfway up, enter halfway Setpoint, enable auto.
 *   Adjust kg such that motor counteracts gravity and lift stays put
 * - Enter a setpoint above current height (or let lift settle below setpoint)
 *   and adjust ks such that motor just barely starts moving 'up'
 * - Enter different setpoints and adjust P(ID) such that lift gets there
 * 
 * .. then update to ProfiledPID?
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
    double voltage = -5.0 * OI.joystick.getRightY();
    lift.setVoltage(voltage);
    SmartDashboard.putNumber("Lift Voltage", voltage);
  }

  @Override
  public void autonomousPeriodic()
  {
    lift.setHeight(SmartDashboard.getNumber("Setpoint", 0.0));
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandBaseRobot;

/** Lift test robot
 *
 *  TODO Setup Procedure:
 * - Use correct motors and sensor types, ID, ..
 *
 * Disabled:
 * - Check if "At Bottom" is correctly indicated,
 *   'true' when at bottom.abstract.
 *   Ideally, wiring could be fail-safe..
 * 
 * Teleop, right stick:
 * - Disconnect secondary motor from speed controller
 * - Check if moving 'up' with positive voltage indeed moves primary motor 'up'.
 *   If not, invert both motors.
 * - Check if moving 'down' stops when hitting bottom switch,
 *   and then only 'up' is possible until clearing the switch?
 * - Check if indicated "Height" zeroes when hitting bottom switch
 * - Calibrate height encoder
 * - Connect secondary motor to speed controller and check that it moves
 *   the same direction as primary. If not, reverse secondary  wiring.
 * - Find good small positive voltage for Lift.PRE_HOMING_VOLTAGE
 *   and negative voltage for Lift.HOMING_VOLTAGE
 * - Test homing: Does holding 'Y' move lift up to pre-home, then stop?
 *   Does then holding 'A' slowly move down until hitting the switch?
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
    if (OI.joystick.getYButton())
      lift.pre_home();
    else if (OI.joystick.getAButton())
      lift.home();
    else
    {
      // For 'up', send position voltage
      double voltage = -5.0 * OI.joystick.getRightY();
      lift.setVoltage(voltage);
      SmartDashboard.putNumber("Voltage", voltage);
    }
  }

  @Override
  public void autonomousPeriodic()
  {
    lift.setHeight(SmartDashboard.getNumber("Setpoint", 0.0));
  }
}

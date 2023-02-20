// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CommandBaseRobot;

/** Grabber test robot
 *
 *  Setup Procedure:
 *  - Use correct motor and sensor IDs
 *  - Enable default 'GrabberOffCommand'
 * 
 *  Disabled:
 *  - Do sensors detect "Cube" or "Cone"?
 * 
 *  Teleop, right stick:
 *  - Check if moving 'up' with positive voltage moves
 *    all spinners to pull game piece 'in'.
 *    If not, reverse motor wiring.
 *  - Determine suitable voltages for pulling in
 *    and pushing out, set Grabber.xxxx_VOLTAGE
 * 
 *  - Does 'X' spin grabber slowly until cube is detected?
 *  - Does 'Y' eject until 1.5 secs after cube is gone?
 *  - Does 'B' spin grabber a bit fater until cone is detected?
 *  - Does 'Y' eject until 1.5 secs after cone is gone?
 */
public class GrabberTestRobot extends CommandBaseRobot
{
  private final Grabber grabber = new Grabber();

  private class ManualCommand extends CommandBase
  {
    public ManualCommand()
    {
      addRequirements(grabber);
    }

    @Override
    public void execute()
    {
      // For 'up', send position voltage
      double voltage = -10.0 * OI.joystick.getRightY();
      grabber.setVoltage(voltage);
      SmartDashboard.putNumber("Voltage", voltage);
    }
  };

  private final CommandBase manual = new ManualCommand();
  private final CommandBase cone   = new GrabConeCommand(grabber);
  private final CommandBase cube   = new GrabCubeCommand(grabber);
  private final CommandBase eject  = new GrabberEjectCommand(grabber);

  @Override
  public void teleopInit()
  {
    manual.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    if (OI.joystick.getXButtonPressed())
      cube.schedule();
    if (OI.joystick.getBButtonPressed())
      cone.schedule();
    if (OI.joystick.getYButtonPressed())
      eject.schedule();
    // If those commands complete,
    // Grabber's default GrabberOffCommand takes over
  }
}

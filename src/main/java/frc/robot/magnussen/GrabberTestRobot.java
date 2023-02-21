// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandBaseRobot;
import frc.robot.util.CycleDelayFilter;

/** Grabber test robot
 *
 *  Setup Procedure:
 *  - Use correct motor and sensor IDs
 *  - Enable default 'GrabberOffCommand'
 * 
 *  Disabled:
 *  - Do sensors detect "Cube" or "Cone"?
 *    NOTE THAT ONLY "CUBE" SENSOR IS USED IN AUTO TEST! 
 * 
 *  Teleop, right stick:
 *  - Check if moving 'up' with positive voltage moves
 *    all spinners to pull game piece 'in'.
 *    If not, reverse motor wiring.
 *  - Determine suitable voltages for pulling in
 *    and pushing out, set GrabVoltage and Grabber.*_VOLTAGE
 * 
 *  Auto:
 *  - Tweak GrabVoltage and GrabDelay to capture a game piece,
 *    hold for 2 secs, then release
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

  private class GrabThingCommand extends CommandBase
  {
    // Keep pulling game piece in a little longer?
    private final CycleDelayFilter delay = new CycleDelayFilter((int)SmartDashboard.getNumber("GrabDelay", 0.0));
    private boolean done;

    public GrabThingCommand()
    {
      addRequirements(grabber);
    }

    @Override
    public void execute()
    {
      done = delay.compute(grabber.haveCube());
      grabber.setVoltage(done ? 0 : SmartDashboard.getNumber("GrabVoltage", 0.0));
    }

    @Override
    public boolean isFinished()
    {
      return done;
    }
  };

  GrabberTestRobot()
  {
    grabber.setDefaultCommand(new ManualCommand());

    SmartDashboard.setDefaultNumber("GrabDelay", 0.0);
    SmartDashboard.setDefaultNumber("GrabVoltage", 0.0);
  }

  @Override
  public void autonomousInit()
  {
    new SequentialCommandGroup(
      new GrabThingCommand(),      
      new WaitCommand(2.0),
      new GrabberEjectCommand(grabber)).schedule();
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to eject game piece from grabber */
public class GrabberEjectCommand extends CommandBase
{
  private final Grabber grabber;
  private final Timer timer = new Timer();

  public GrabberEjectCommand(Grabber grabber)
  {
    this.grabber = grabber;
    addRequirements(grabber);
  }

  @Override
  public void initialize()
  {
    timer.restart();
  }

  @Override
  public void execute()
  {
    grabber.setVoltage(Grabber.EJECT_VOLTAGE);
  }

  @Override
  public boolean isFinished()
  {
    return timer.hasElapsed(1.5);
  }
}

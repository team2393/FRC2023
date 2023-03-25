// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to grab cone */
public class GrabConeCommand extends CommandBase
{
  private final Grabber grabber;
  // Keep pulling game piece in a little longer?
  private final Timer timer = new Timer();

  private boolean done;

  public GrabConeCommand(Grabber grabber)
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
    if (! grabber.haveGamepiece())
    {
      timer.restart();
      done = false;
    }
    else
    {
      done = timer.hasElapsed(1.0);
    }
    grabber.setVoltage(done ? 0 : Grabber.CONE_VOLTAGE);
  }

  @Override
  public boolean isFinished()
  {
    return done;
  }
}

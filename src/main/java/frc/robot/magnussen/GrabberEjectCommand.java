// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.CycleDelayFilter;

/** Command to eject game piece from grabber */
public class GrabberEjectCommand extends CommandBase
{
  private final Grabber grabber;
  // Option to keep ejecting after game pieces no longer detected
  private final CycleDelayFilter delay = CycleDelayFilter.forSeconds(1.5);
  private boolean done;

  public GrabberEjectCommand(Grabber grabber)
  {
    this.grabber = grabber;
    addRequirements(grabber);
  }

  @Override
  public void execute()
  {
    // Keep ejecting until we have no cube and no cone
    // - same as   !(grabber.haveCube() || grabber.haveCone())
    // .. and then keep ejecting a little longer just to make sure
    //    game piece is gone
    done = delay.compute(!grabber.haveCube()   &&   !grabber.haveCone());
    grabber.setVoltage(done ? 0 : Grabber.EJECT_VOLTAGE);
  }

  @Override
  public boolean isFinished()
  {
    return done;
  }
}

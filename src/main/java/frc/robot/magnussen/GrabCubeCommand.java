// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.CycleDelayFilter;

/** Command to grab cube */
public class GrabCubeCommand extends CommandBase
{
  private final Grabber grabber;
  // Keep pulling game piece in a little longer?
  private final CycleDelayFilter delay = new CycleDelayFilter(20);
  private boolean done;

  public GrabCubeCommand(Grabber grabber)
  {
    this.grabber = grabber;
    addRequirements(grabber);
  }

  @Override
  public void initialize()
  {
    grabber.setCubeLimit();
  }

  @Override
  public void execute()
  {
    done = delay.compute(grabber.haveGamepiece());
    grabber.setVoltage(done ? 0 : Grabber.CUBE_VOLTAGE);
  }

  @Override
  public boolean isFinished()
  {
    return done;
  }
}

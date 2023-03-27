// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to keep grabber pulling in */
public class GrabberKeepCommand extends CommandBase
{
  private final Grabber grabber;

  public GrabberKeepCommand(Grabber grabber)
  {
    this.grabber = grabber;
    addRequirements(grabber);
  }

  @Override
  public void initialize()
  {
    System.out.println("KEEP IN!!");
  }
  @Override
  public void execute()
  {
    grabber.setVoltage(2.0);
  }
}

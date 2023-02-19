// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to keep grabber motor off */
public class GrabberOffCommand extends CommandBase
{
  private final Grabber grabber;

  public GrabberOffCommand(Grabber grabber)
  {
    this.grabber = grabber;
    addRequirements(grabber);
  }

  @Override
  public void execute()
  {
    grabber.setVoltage(0);
  }
}

// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Assert that intake is in idle position, arm in front of it.
  *  If not, move everything to idle position
  */
public class MakeSafeCommand extends CommandBase
{
  private final Charm coordinator;
  private CommandBase sub_command;

  /** @param coordinator Coordinator */
  public MakeSafeCommand(Charm coordinator)
  {
    this.coordinator = coordinator;
  }
  
  @Override
  public void initialize()
  {
    if (coordinator.arm_setpoint > -100)
      sub_command = null;
    else
    { // Only create idle commands when needed
      sub_command = coordinator.idle();
      sub_command.initialize();
    }
  }

  @Override
  public void execute()
  {
    if (sub_command != null)
      sub_command.execute();
  }

  @Override
  public void end(boolean interrupted)
  {
    if (sub_command != null)
      sub_command.end(interrupted);
  }

  @Override
  public boolean isFinished()
  {
    return sub_command == null   ||  sub_command.isFinished();
  }
}
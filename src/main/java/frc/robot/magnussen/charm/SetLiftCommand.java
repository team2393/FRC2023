// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import static java.lang.Math.abs;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLiftCommand extends CommandBase
{
  private Charm coordinator;
  private double height;
    
  /** @param coordinator Coordinator
   *  @param height Desired lift height, wait until within 3 cm
   */
  public SetLiftCommand(Charm coordinator, double height)
  {
    this.coordinator = coordinator;
    this.height = height;
  }

  @Override
  public void initialize()
  {
    coordinator.lift_setpoint = height;
  }

  @Override
  public boolean isFinished()
  {
    return abs(height - coordinator.lift.getHeight()) < 0.03;
  }
}
// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import static java.lang.Math.abs;
import static java.lang.Math.IEEEremainder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeCommand extends CommandBase
{
  private Charm coordinator;
  private double desired;
    
  /** @param coordinator Coordinator
   *  @param desired Desired intake angle, wait until within 5 degrees
   */
  public SetIntakeCommand(Charm coordinator, double desired)
  {
    this.coordinator = coordinator;
    this.desired = desired;
  }

  @Override
  public void initialize()
  {
    coordinator.intake_setpoint = desired;
  }

  @Override
  public boolean isFinished()
  {
      return abs(IEEEremainder(desired - coordinator.intake.getAngle(), 360)) < 5.0;
  }
}
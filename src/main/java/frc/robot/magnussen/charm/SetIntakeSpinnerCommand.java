// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeSpinnerCommand extends InstantCommand
{
  /** @param coordinator Coordinator
   *  @param voltage Desired spinner voltage
   */
  public SetIntakeSpinnerCommand(Charm coordinator, double voltage)
  {
    super(() -> coordinator.intake.setSpinner(voltage));
  }
}
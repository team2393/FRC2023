// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.magnussen.Intake;

public class SetIntakeSpinnerCommand extends InstantCommand
{
  /** @param intake Intake
   *  @param voltage Desired spinner voltage
   */
  public SetIntakeSpinnerCommand(Intake intake, double voltage)
  {
    super(() -> intake.setSpinner(voltage));
  }
}
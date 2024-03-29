// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen.charm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RetractArmCommand extends InstantCommand
{
  /** @param coordinator Coordinator */
  public RetractArmCommand(Charm coordinator)
  {
    super(() -> coordinator.arm.extend(false));
  }
}
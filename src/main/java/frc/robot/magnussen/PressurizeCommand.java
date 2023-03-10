// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.magnussen;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that awaits sufficient air pressure, once */
public class PressurizeCommand extends CommandBase
{
  private static boolean pressurized = false;
  
  @Override
  public boolean isFinished()
  {
    if (! pressurized)
      pressurized = SmartDashboard.getNumber("Pressure", 0.0) > 80.0;
    return pressurized;
  }
}